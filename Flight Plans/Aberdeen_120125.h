#define WP_RADIUS 75 // What is the minimum distance to reach a waypoint?
#define LOITER_RADIUS 125 	// How close to Loiter?
#define HOLD_CURRENT_ALT 0	// 1 = hold the current altitude, 0 = use the defined altitude to for RTL
#define ALT_TO_HOLD 200

float mission[][5] = {
{WAYPOINT,0,200,37.0222226,-76.5893412},
{WAYPOINT,0,200,37.0222568,-76.5917444},
{WAYPOINT,0,200,37.0196533,-76.5930938},
{DO_JUMP,1,0,9999977,0},
{WAYPOINT,0,200,37.0195846,-76.5924301},
};
