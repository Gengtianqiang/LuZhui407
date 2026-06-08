#ifndef MESH_CMD_H
#define MESH_CMD_H

/* KEY */
#define MESH_KEY_NONE   0x0FU

/* 4~0 bit */
#define MESH_KEY_UP     1U
#define MESH_KEY_DOWN   2U
#define MESH_KEY_LEFT   3U
#define MESH_KEY_RIGHT  4U

#define MESH_KEY_MODE    5U
#define MESH_KEY_SELECT  6U
#define MESH_KEY_START   7U

#define MESH_KEY_TRANGLE  8U
#define MESH_KEY_FORK     9U
#define MESH_KEY_SQUARE   10U
#define MESH_KEY_CIRCLE   11U

#define MESH_KEY_L1   12U
#define MESH_KEY_L2   13U
#define MESH_KEY_R1   14U
#define MESH_KEY_R2   15U

#define MESH_KEY_L3   16U
#define MESH_KEY_R3   17U


/* for func */
/* left shift range from 7~5 bit */
//0x47
#define MESH_FUNC_FIND_NODE     (1U << 6)|MESH_KEY_START
//0x25
#define MESH_FUNC_RREE_CONTROL  (1U << 5)|MESH_KEY_MODE


/* define */
#define     MESH_NETID      "AT+NETID0A19132E,6\r\n"

#define MESH_FUNC_FIND_NODE_TIMEOUT  4000u
#define MESH_SENDDATA_TIME_INTERVAL  100u



#endif
