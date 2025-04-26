idf_component_register(SRCS "scd4x.c" # relevant source files
                    REQUIRES driver # required because scd4x.h includes "driver/i2c.h"
                    INCLUDE_DIRS ".") # directories to add to the global search path


