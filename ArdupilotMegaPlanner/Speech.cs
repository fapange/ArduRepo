﻿
#define SLV_ADDED

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Speech.Synthesis;
using log4net;

namespace ArdupilotMega
{
    public class Speech
    {
        private static readonly ILog log = LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

        SpeechSynthesizer _speechwindows;
        System.Diagnostics.Process _speechlinux;

        System.Speech.Synthesis.SynthesizerState _state = SynthesizerState.Ready;

        bool MONO = false;

        public SynthesizerState State
        {
            get
            {
                if (MONO)
                {
                    return _state;
                }
                else
                {
                    return _speechwindows.State;
                }
            }
        }

        public Speech()
        {
            var t = Type.GetType("Mono.Runtime");
            MONO = (t != null);

            log.Info("TTS: init, mono = " + MONO);

            if (MONO)
            {
                _state = SynthesizerState.Ready;
            }
            else
            {
                _speechwindows = new SpeechSynthesizer();
            }
        }

        public void SpeakAsync(string text)
        {
#if SLV_ADDED
            string cleanText = text;
            cleanText = cleanText.Replace("<", " ");
            cleanText = cleanText.Replace(">", " ");
            cleanText = cleanText.Replace("-", " ");
            cleanText = cleanText.Replace("_", " ");
            cleanText = cleanText.Replace(":", " ");
            cleanText = cleanText.Replace("!", " ");
            cleanText = cleanText.Replace(".", " ");
            cleanText = cleanText.Replace("\n", " ");
#endif
            if (MONO)
            {
                //if (_speechlinux == null)
                {
                    _state = SynthesizerState.Speaking;
                    _speechlinux = new System.Diagnostics.Process();
                    _speechlinux.StartInfo.RedirectStandardInput = true;
                    _speechlinux.StartInfo.UseShellExecute = false;
                    _speechlinux.StartInfo.FileName = "festival";
                    _speechlinux.Start();
                    _speechlinux.Exited += new EventHandler(_speechlinux_Exited);

                    log.Info("TTS: start " + _speechlinux.StartTime);

                }

                _state = SynthesizerState.Speaking;
#if SLV_ADDED
                _speechlinux.StandardInput.WriteLine("(SayText \"" + cleanText + "\")");
#else
                _speechlinux.StandardInput.WriteLine("(SayText \"" + text + "\")");
#endif
                _speechlinux.StandardInput.WriteLine("(quit)");

                _speechlinux.Close();

                _state = SynthesizerState.Ready;
            }
            else
            {
#if SLV_ADDED
                _speechwindows.SpeakAsync(cleanText);
#else
                _speechwindows.SpeakAsync(text);
#endif
            }

#if SLV_ADDED
            log.Info("TTS: say " + cleanText);
#else
            log.Info("TTS: say " + text);
#endif
        }

        void _speechlinux_Exited(object sender, EventArgs e)
        {
            log.Info("TTS: exit " + _speechlinux.ExitTime);
            _state = SynthesizerState.Ready;
        }

        public void SpeakAsyncCancelAll()
        {
            if (MONO)
            {
                _speechlinux.Close();
                _state = SynthesizerState.Ready;
            }
            else
            {
                _speechwindows.SpeakAsyncCancelAll();
            }
        }
    }
}