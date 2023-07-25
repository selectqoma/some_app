# The name of the tag
string name

# The id of the tag that's appended to the server's url
string id

# The value of the tag
string value

# The data type of the value used to parse it from a string
uint8 VALUE_TYPE_INT = 0
uint8 VALUE_TYPE_FLOAT = 1
uint8 VALUE_TYPE_STRING = 2
uint8 VALUE_TYPE_BOOL = 3
uint8 VALUE_TYPE_TIME = 4
uint8 value_type
