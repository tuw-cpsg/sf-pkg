# Header including a message ID, a timestamp and a frame ID.
Header header
# Estimated value of the entity.
float64 value
# Variance, i.e. possible deviation, of the value.
float64 variance
# Jitter from the time of sensing and sending this message.
uint32 jitter_ms