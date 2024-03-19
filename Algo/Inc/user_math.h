#ifndef USER_MATH_H
#define USER_MATH_H

#define PI (3.1415926f)

#define __MAX_LIMIT(val, min, max)      do\
                                        {\
                                            val =(val > max ? max : val);\
                                            val = (val < min ? min : val);\
                                        }\
                                        while(0);

#define DEG_TO_RAD 3.14159f / 180.0f

#define __MAP(x, in_min, in_max, out_min, out_max) do\
                                                { \
                                                if (x > in_max) { x -= in_max; } \
                                                else if (x < -in_max) {x += in_max;} \
                                                x = ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);\
                                                } \
                                                while(0);
                    
#define __MAP_ANGLE_TO_UNIT_CIRCLE(x) do\
                                    {\
                                        while (error >= PI) { error -= 2 * PI; }\
                                        while (error < -PI) { error += 2 * PI; }\
                                    }\
                                    while(0);
#endif
