/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _util_h_
#define _util_h_

#include <string>
#include <vector>

#define FLOAT_LEN 4  // sizeof(float)
#define DOUBLE_LEN 8 // sizeof(double)
#define CHAR_LEN 1   // sizeof(char)
#define SHORT_LEN 2  // sizeof(short)
#define INT_LEN 4    // sizeof(int)
#define LONG_LEN 4   // sizeof(long)

// ----------------------------------
// Deserialize methods
// ----------------------------------
/**
 * Deserialize float data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
float deserialize_float(const unsigned char *buffer, unsigned int &length);

/**
 * Deserialize double data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
double deserialize_double(const unsigned char *buffer, unsigned int &length);

/**
 * Deserialize unsigned char data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
unsigned char deserialize_unsigned_char(const unsigned char *buffer,
                                        unsigned int &length);

/**
 * Deserialize unsigned short data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
unsigned short deserialize_unsigned_short(const unsigned char *buffer,
                                          unsigned int &length);

/**
 * Deserialize unsigned int data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
unsigned int deserialize_unsigned_int(const unsigned char *buffer,
                                      unsigned int &length);

/**
 * Deserialize unsigned long data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
unsigned long deserialize_unsigned_long(const unsigned char *buffer,
                                        unsigned int &length);

/**
 * Deserialize char data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
char deserialize_char(const unsigned char *buffer, unsigned int &length);

/**
 * Deserialize short data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
short deserialize_short(const unsigned char *buffer, unsigned int &length);

/**
 * Deserialize int data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
int deserialize_int(const unsigned char *buffer, unsigned int &length);

/**
 * Deserialize long data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
long deserialize_long(const unsigned char *buffer, unsigned int &length);

/**
 * Deserialize std::string data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::string deserialize_string(const unsigned char *buffer,
                               unsigned int &length);

// ----------------------------------
// Deserialize each data type array methods
// ----------------------------------
/**
 * Deserialize std::vector<unsigned char> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<unsigned char>
deserialize_array_unsigned_char(const unsigned char *buffer,
                                unsigned int &length);
/**
 * Deserialize std::vector<unsigned short> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<unsigned short>
deserialize_array_unsigned_short(const unsigned char *buffer,
                                 unsigned int &length);
/**
 * Deserialize std::vector<unsigned int> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<unsigned int>
deserialize_array_unsigned_int(const unsigned char *buffer,
                               unsigned int &length);
/**
 * Deserialize std::vector<unsigned long> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<unsigned long>
deserialize_array_unsigned_long(const unsigned char *buffer,
                                unsigned int &length);
/**
 * Deserialize std::vector<char> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<char> deserialize_array_char(const unsigned char *buffer,
                                         unsigned int &length);
/**
 * Deserialize std::vector<short> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<short> deserialize_array_short(const unsigned char *buffer,
                                           unsigned int &length);
/**
 * Deserialize std::vector<int> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<int> deserialize_array_int(const unsigned char *buffer,
                                       unsigned int &length);
/**
 * Deserialize std::vector<long> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<long> deserialize_array_long(const unsigned char *buffer,
                                         unsigned int &length);
/**
 * Deserialize std::vector<float> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<float> deserialize_array_float(const unsigned char *buffer,
                                           unsigned int &length);
/**
 * Deserialize std::vector<double> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<double> deserialize_array_double(const unsigned char *buffer,
                                             unsigned int &length);
/**
 * Deserialize std::vector<std::string> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<std::string> deserialize_array_string(const unsigned char *buffer,
                                                  unsigned int &length);
// ----------------------------------

// ----------------------------------
// Serialize methods
// ----------------------------------
/**
 * Serialize float data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_float(float number, unsigned char *buffer);

/**
 * Serialize double data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_double(double number, unsigned char *buffer);

/**
 * Serialize unsigned char data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_unsigned_char(unsigned char number,
                                     unsigned char *buffer);

/**
 * Serialize unsigned short data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_unsigned_short(unsigned short number,
                                      unsigned char *buffer);

/**
 * Serialize unsigned int data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_unsigned_int(unsigned int number, unsigned char *buffer);

/**
 * Serialize unsigned long data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_unsigned_long(unsigned long number,
                                     unsigned char *buffer);

/**
 * Serialize char data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_char(char number, unsigned char *buffer);

/**
 * Serialize short data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_short(short number, unsigned char *buffer);

/**
 * Serialize int data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_int(int number, unsigned char *buffer);

/**
 * Serialize long data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_long(long number, unsigned char *buffer);

/**
 * Serialize std::string data.
 * @param str Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_string(const std::string &str, unsigned char *buffer);

// ----------------------------------
// Serialize each data type array methods
// ----------------------------------
/**
 * Serialize std::vector<unsigned char> data.
 * @param array     Data to be serialized 
 * @param buffer    Serialized data
 * @return Data length
 */
unsigned int
serialize_array_unsigned_char(const std::vector<unsigned char> &array,
                              unsigned char *buffer);
/**
 * Serialize std::vector<unsigned short> data.
 * @param array     Data to be serialized 
 * @param buffer    Serialized data
 * @return Data length
 */
unsigned int
serialize_array_unsigned_short(const std::vector<unsigned short> &array,
                               unsigned char *buffer);
/**
 * Serialize std::vector<unsigned int> data.
 * @param array     Data to be serialized 
 * @param buffer    Serialized data
 * @return Data length
 */
unsigned int
serialize_array_unsigned_int(const std::vector<unsigned int> &array,
                             unsigned char *buffer);
/**
 * Serialize std::vector<unsigned long> data.
 * @param array     Data to be serialized 
 * @param buffer    Serialized data
 * @return Data length
 */
unsigned int
serialize_array_unsigned_long(const std::vector<unsigned long> &array,
                              unsigned char *buffer);
/**
 * Serialize std::vector<char> data.
 * @param array     Data to be serialized 
 * @param buffer    Serialized data
 * @return Data length
 */
unsigned int serialize_array_char(const std::vector<char> &array,
                                  unsigned char *buffer);
/**
 * Serialize std::vector<short> data.
 * @param array     Data to be serialized 
 * @param buffer    Serialized data
 * @return Data length
 */
unsigned int serialize_array_short(const std::vector<short> &array,
                                   unsigned char *buffer);
/**
 * Serialize std::vector<int> data.
 * @param array     Data to be serialized 
 * @param buffer    Serialized data
 * @return Data length
 */
unsigned int serialize_array_int(const std::vector<int> &array,
                                 unsigned char *buffer);
/**
 * Serialize std::vector<long> data.
 * @param array     Data to be serialized 
 * @param buffer    Serialized data
 * @return Data length
 */
unsigned int serialize_array_long(const std::vector<long> &array,
                                  unsigned char *buffer);
/**
 * Serialize std::vector<float> data.
 * @param array     Data to be serialized 
 * @param buffer    Serialized data
 * @return Data length
 */
unsigned int serialize_array_float(const std::vector<float> &array,
                                   unsigned char *buffer);
/**
 * Serialize std::vector<std::string> data.
 * @param array     Data to be serialized 
 * @param buffer    Serialized data
 * @return Data length
 */
unsigned int serialize_array_string(const std::vector<std::string> &array,
                                    unsigned char *buffer);
/**
 * Serialize std::vector<double> data.
 * @param array     Data to be serialized 
 * @param buffer    Serialized data
 * @return Data length
 */
unsigned int serialize_array_double(const std::vector<double> &array,
                                    unsigned char *buffer);
// ----------------------------------

#endif // _util_h_