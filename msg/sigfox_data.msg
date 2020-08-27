# Sigfox data is used to send by Sigfox module to Sigfox network

uint64 timestamp			# time since system start (microseconds)
bool bstartsend				# Control sigfox_sender to start sending
uint8 unformat				# Set sigfox message format
							# 0: readable format. (0XX.XXXXAXXX.XXXXAXXXX -> [Latitude]A[Longitude]A[Altitude])
							# 1: bytes format. (XXXXXXYYYYYYZZZZ -> [Latitude 3 bytes]A[Longitude 3 bytes]A[Altitude 2 bytes])

# GPS Data which was sent
char[25] clastsentmsg		# The last message string sent through Sigfox (Max size is 24)