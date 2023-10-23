#ifndef USER_MATH_H
#define USER_MATH_H

#define __MAX_LIMIT(val, min, max)      do\
                                        {\
                                            val =(val > max ? max : val);\
                                            val = (val < min ? min : val);\
                                        }\
                                        while(0);

#define DEG_TO_RAD 3.14159f / 180.0f

#endif
