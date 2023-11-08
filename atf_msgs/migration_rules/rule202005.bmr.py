class update_atf_msgs_MetricResult_438a6e6fcdba4c073c5c940bc183fe12(MessageUpdateRule):
	old_type = "atf_msgs/MetricResult"
	old_full_text = """
uint8 SNAP        = 0
uint8 SPAN        = 1

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

================================================================================
MSG: atf_msgs/DataStamped
time stamp
float64 data

================================================================================
MSG: atf_msgs/Groundtruth
bool available
bool result
string error_message
float64 data
float64 epsilon

================================================================================
MSG: atf_msgs/KeyValue
string key
string value
"""

	new_type = "atf_msgs/MetricResult"
	new_full_text = """
uint8 SNAP        = 0
uint8 SPAN_MEAN   = 1
uint8 SPAN_MIN    = 2
uint8 SPAN_ABSMIN = 3
uint8 SPAN_MAX    = 4
uint8 SPAN_ABSMAX = 5

string name
string unit
uint8 mode
int8 status # ENUM from TestblockStatus
DataStamped[] series
DataStamped data
DataStamped min
DataStamped max
float64 mean
float64 std
Groundtruth groundtruth
KeyValue[] details

================================================================================
MSG: atf_msgs/DataStamped
time stamp
float64 data

================================================================================
MSG: atf_msgs/Groundtruth
# result status
int8 FAILED      = -1
int8 UNSET       = 0
int8 SUCCEEDED   = 1

bool available
int8 result
string error_message
float64 data
float64 epsilon

================================================================================
MSG: atf_msgs/KeyValue
string key
string value
"""

	order = 0
	migrated_types = [
		("DataStamped","DataStamped"),
		("Groundtruth","Groundtruth"),
		("KeyValue","KeyValue"),]

	valid = True

	def update(self, old_msg, new_msg):
		#Constant 'SPAN' has changed
		new_msg.name = old_msg.name
		#No matching field name in old message
		new_msg.unit = ''
		new_msg.mode = old_msg.mode
		#No matching field name in old message
		new_msg.status = 0
		self.migrate_array(old_msg.series, new_msg.series, "atf_msgs/DataStamped")
		self.migrate(old_msg.data, new_msg.data)
		self.migrate(old_msg.min, new_msg.min)
		self.migrate(old_msg.max, new_msg.max)
		new_msg.mean = old_msg.mean
		new_msg.std = old_msg.std
		self.migrate(old_msg.groundtruth, new_msg.groundtruth)
		self.migrate_array(old_msg.details, new_msg.details, "atf_msgs/KeyValue")
		#No field to match field started from old message
		#No field to match field finished from old message
class update_atf_msgs_Groundtruth_adbb6bb4eb068fd5af7659b413397d10(MessageUpdateRule):
	old_type = "atf_msgs/Groundtruth"
	old_full_text = """
bool available
bool result
string error_message
float64 data
float64 epsilon
"""

	new_type = "atf_msgs/Groundtruth"
	new_full_text = """
# result status
int8 FAILED      = -1
int8 UNSET       = 0
int8 SUCCEEDED   = 1

bool available
int8 result
string error_message
float64 data
float64 epsilon
"""

	order = 0
	migrated_types = []

	valid = True

	def update(self, old_msg, new_msg):
		new_msg.available = old_msg.available
		#Primitive type changed
		if old_msg.result:
			new_msg.result = 1
		else:
			new_msg.result = -1
		new_msg.error_message = old_msg.error_message
		new_msg.data = old_msg.data
		new_msg.epsilon = old_msg.epsilon
class update_atf_msgs_TestblockStatus_361a3a189cc39d4afd9ca41fbafd986b(MessageUpdateRule):
	old_type = "atf_msgs/TestblockStatus"
	old_full_text = """
int8 INACTIVE    = -1
int8 PURGED      = 0
int8 ACTIVE      = 1
int8 PAUSED      = 2
int8 SUCCEEDED   = 3
int8 ERROR       = 4

time stamp
string name
int8 status
MetricResult user_result

================================================================================
MSG: atf_msgs/MetricResult
uint8 SNAP        = 0
uint8 SPAN        = 1

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

================================================================================
MSG: atf_msgs/DataStamped
time stamp
float64 data

================================================================================
MSG: atf_msgs/Groundtruth
bool available
bool result
string error_message
float64 data
float64 epsilon

================================================================================
MSG: atf_msgs/KeyValue
string key
string value
"""

	new_type = "atf_msgs/TestblockStatus"
	new_full_text = """
int8 INACTIVE    = 0
int8 ACTIVE      = 1
int8 PAUSED      = 2
int8 PURGED      = 3
int8 SUCCEEDED   = 4
int8 ERROR       = 5

time stamp
string name
int8 status
MetricResult user_result

================================================================================
MSG: atf_msgs/MetricResult
uint8 SNAP        = 0
uint8 SPAN_MEAN   = 1
uint8 SPAN_MIN    = 2
uint8 SPAN_ABSMIN = 3
uint8 SPAN_MAX    = 4
uint8 SPAN_ABSMAX = 5

string name
string unit
uint8 mode
int8 status # ENUM from TestblockStatus
DataStamped[] series
DataStamped data
DataStamped min
DataStamped max
float64 mean
float64 std
Groundtruth groundtruth
KeyValue[] details

================================================================================
MSG: atf_msgs/DataStamped
time stamp
float64 data

================================================================================
MSG: atf_msgs/Groundtruth
# result status
int8 FAILED      = -1
int8 UNSET       = 0
int8 SUCCEEDED   = 1

bool available
int8 result
string error_message
float64 data
float64 epsilon

================================================================================
MSG: atf_msgs/KeyValue
string key
string value
"""

	order = 0
	migrated_types = [
		("MetricResult","MetricResult"),]

	valid = True

	def update(self, old_msg, new_msg):
		#Constant 'SUCCEEDED' has changed
		#Constant 'PURGED' has changed
		#Constant 'INACTIVE' has changed
		#Constant 'ERROR' has changed
		new_msg.stamp = old_msg.stamp
		new_msg.name = old_msg.name
		status_dict = {
			-1:0,
			0:3,
			1:1,
			2:2,
			3:4,
			4:5
		}
		new_msg.status = status_dict[old_msg.status]
		self.migrate(old_msg.user_result, new_msg.user_result)
