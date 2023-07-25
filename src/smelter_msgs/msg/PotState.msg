Header header
uint8 id

int8 AREA_UNKNOWN = -1
int8 AREA_CASCADE = 0
int8 AREA_SLAGSQUARE = 1
int8 AREA_CRANE = 2
int8 AREA_DUMP = 3
int8 area

int8 POSITION_TYPE_UNKNOWN = -1
int8 POSITION_TYPE_2D_GRID = 0
int8 POSITION_TYPE_2D_METRIC = 1
int8 POSITION_TYPE_3D_METRIC = 2
int8 position_type

geometry_msgs/Point position

int8 FILLING_LEVEL_UNKNOWN = -1
int8 FILLING_LEVEL_EMPTY = 0
int8 FILLING_LEVEL_HALF_FULL = 1
int8 FILLING_LEVEL_FULL = 2
int8 filling_level

int8 COMPOSITION_UNKNOWN = -1
int8 COMPOSITION_SLAG = 0
int8 COMPOSITION_MATTE = 1
int8 COMPOSITION_FOAM = 2
int8 composition

sensor_msgs/Temperature[] temperature_history
