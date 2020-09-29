uint8 SNAP        = 0
uint8 SPAN_MEAN   = 1
uint8 SPAN_MIN    = 2
uint8 SPAN_ABSMIN = 3
uint8 SPAN_MAX    = 4
uint8 SPAN_ABSMAX = 5

string name
uint8 mode
bool started
bool finished
DataStamped[] series
DataStamped data
DataStamped min
DataStamped max
float64 mean
float64 std
Groundtruth groundtruth
KeyValue[] details
