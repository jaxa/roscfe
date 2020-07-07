/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/

/*************************************************************************
** Includes
*************************************************************************/
#include "util.h"

/**
 * Deserialize float data. 
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
float deserialize_float(const unsigned char *buffer, unsigned int &length) {
  unsigned int ii = 0;
  length = FLOAT_LEN;
  unsigned char read_buffer[FLOAT_LEN];
  for (ii = 0; ii < length; ii++) {
    read_buffer[ii] = buffer[ii];
  }
  float *value = reinterpret_cast<float*>(read_buffer);
  return *value;
}

/**
 * Deserialize double data. 
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
double deserialize_double(const unsigned char *buffer, unsigned int &length) {
  unsigned int ii = 0;
  length = DOUBLE_LEN;
  unsigned char read_buffer[DOUBLE_LEN];
  for (ii = 0; ii < length; ii++) {
    read_buffer[ii] = buffer[ii];
  }
  double *value = reinterpret_cast<double*>(read_buffer);
  return *value;
}

/**
 * Deserialize unsigned char data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
unsigned char deserialize_unsigned_char(const unsigned char *buffer, unsigned int &length) {
  unsigned char value = 0;
  unsigned int ii;
  length = CHAR_LEN;
  for (ii = 0; ii < length; ii++) {
    value += (unsigned char)(buffer[ii] << (ii * 8));
  }
  return value;
}

/**
 * Deserialize unsigned short data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
unsigned short deserialize_unsigned_short(const unsigned char *buffer, unsigned int &length) {
  unsigned short value = 0;
  unsigned int ii;
  length = SHORT_LEN;
  for (ii = 0; ii < length; ii++) {
    value += (unsigned short)(buffer[ii] << (ii * 8));
  }
  return value;
}

/**
 * Deserialize unsigned int data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
unsigned int deserialize_unsigned_int(const unsigned char *buffer, unsigned int &length) {
  unsigned int value = 0;
  unsigned int ii;
  length = INT_LEN;
  for (ii = 0; ii < length; ii++) {
    value += (unsigned int)(buffer[ii] << (ii * 8));
  }
  return value;
}

/**
 * Deserialize unsigned long data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
unsigned long deserialize_unsigned_long(const unsigned char *buffer, unsigned int &length) {
  unsigned long value = 0;
  unsigned int ii;
  unsigned int ret = deserialize_unsigned_int(buffer, length);
  value = static_cast<unsigned long>(ret);
  return value;
}

/**
 * Deserialize char data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
char deserialize_char(const unsigned char *buffer, unsigned int &length) {
  char value = 0;
  unsigned int ii;
  length = CHAR_LEN;
  for (ii = 0; ii < length; ii++) {
    value += (char)(buffer[ii] << (ii * 8));
  }
  return value;
}

/**
 * Deserialize short data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
short deserialize_short(const unsigned char *buffer, unsigned int &length) {
  short value = 0;
  unsigned int ii;
  length = SHORT_LEN;
  for (ii = 0; ii < length; ii++) {
    value += (short)(buffer[ii] << (ii * 8));
  }
  return value;
}

/**
 * Deserialize int data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
int deserialize_int(const unsigned char *buffer, unsigned int &length) {
  int value = 0;
  unsigned int ii;
  length = INT_LEN;
  for (ii = 0; ii < length; ii++) {
    value += (int)(buffer[ii] << (ii * 8));
  }
  return value;
}

/**
 * Deserialize long data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
long deserialize_long(const unsigned char *buffer, unsigned int &length) {
  long value = 0;
  unsigned int ii;
  int ret = deserialize_int(buffer, length);
  value = static_cast<long>(ret);

  return value;
}

/**
 * Deserialize std::string data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::string deserialize_string(const unsigned char *buffer, unsigned int &length) {
  unsigned int read_len = 0;
  length = 0;
  const unsigned char *read_str_buffer;
  char *read_chars;
  unsigned int ii;

  // deserialize string length
  unsigned int read_str_len = 0;
  read_str_len = deserialize_unsigned_int(buffer, read_len);
  length += read_len;
  
  // deserialize string
  read_str_buffer = buffer + INT_LEN;
  length += read_str_len;
  read_chars = new char[read_str_len];
  for (ii = 0; ii < read_str_len; ii++) {
    read_chars[ii] = (char)read_str_buffer[ii];
  }
  std::string reconst_str = std::string(read_chars);

  delete[] read_chars;
  return reconst_str;
}

/**
 * Deserialize std::vector<unsigned char> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<unsigned char> deserialize_array_unsigned_char(const unsigned char *buffer, unsigned int &length) {
  std::vector<unsigned char> array;
  unsigned int read_len = 0;
  length = 0;
  unsigned int cell_len = 0;
  unsigned int ii;

  // read array size
  unsigned int size = deserialize_unsigned_int(buffer + length, read_len);
  length += read_len;

  // deserilize array
  for (ii = 0; ii < size; ii++) {
    unsigned char value = deserialize_unsigned_char(buffer + length, cell_len);
    array.push_back(value);
    length += cell_len;
  }
  return array;
}

/**
 * Deserialize std::vector<unsigned short> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<unsigned short> deserialize_array_unsigned_short(const unsigned char *buffer, unsigned int &length) {
  std::vector<unsigned short> array;
  unsigned int read_len = 0;
  length = 0;
  unsigned int cell_len = 0;
  unsigned int ii;

  // read array size
  unsigned int size = deserialize_unsigned_int(buffer, read_len);
  length += read_len;

  // deserilize array
  for (ii = 0; ii < size; ii++) {
    unsigned short value = deserialize_unsigned_short(buffer + length, cell_len);
    array.push_back(value);
    length += cell_len;
  }
  return array;
}

/**
 * Deserialize std::vector<unsigned int> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<unsigned int> deserialize_array_unsigned_int(const unsigned char *buffer, unsigned int &length) {
  std::vector<unsigned int> array;
  unsigned int read_len = 0;
  length = 0;
  unsigned int cell_len = 0;
  unsigned int ii;

  // read array size
  unsigned int size = deserialize_unsigned_int(buffer, read_len);
  length += read_len;

  // deserilize array
  for (ii = 0; ii < size; ii++) {
    unsigned int value = deserialize_unsigned_int(buffer + length, cell_len);
    array.push_back(value);
    length += cell_len;
  }
  return array;
}

/**
 * Deserialize std::vector<unsigned long> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<unsigned long> deserialize_array_unsigned_long(const unsigned char *buffer, unsigned int &length) {
  std::vector<unsigned long> array;
  unsigned int read_len = 0;
  length = 0;
  unsigned int cell_len = 0;
  unsigned int ii;

  // read array size
  unsigned int size = deserialize_unsigned_int(buffer, read_len);
  length += read_len;

  // deserilize array
  for (ii = 0; ii < size; ii++) {
    unsigned long value = deserialize_unsigned_long(buffer + length, cell_len);
    array.push_back(value);
    length += cell_len;
  }
  return array;
}

/**
 * Deserialize std::vector<char> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<char> deserialize_array_char(const unsigned char *buffer, unsigned int &length) {
  std::vector<char> array;
  unsigned int read_len = 0;
  length = 0;
  unsigned int cell_len = 0;
  unsigned int ii;

  // read array size
  unsigned int size = deserialize_unsigned_int(buffer, read_len);
  length += read_len;

  // deserilize array
  for (ii = 0; ii < size; ii++) {
    char value = deserialize_char(buffer + length, cell_len);
    array.push_back(value);
    length += cell_len;
  }
  return array;
}

/**
 * Deserialize std::vector<short> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<short> deserialize_array_short(const unsigned char *buffer, unsigned int &length) {
  std::vector<short> array;
  unsigned int read_len = 0;
  length = 0;
  unsigned int cell_len = 0;
  unsigned int ii;

  // read array size
  unsigned int size = deserialize_unsigned_int(buffer, read_len);
  length += read_len;

  // deserilize array
  for (ii = 0; ii < size; ii++) {
    short value = deserialize_short(buffer + length, cell_len);
    array.push_back(value);
    length += cell_len;
  }
  return array;
}

/**
 * Deserialize std::vector<int> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<int> deserialize_array_int(const unsigned char *buffer, unsigned int &length) {
  std::vector<int> array;
  unsigned int read_len = 0;
  length = 0;
  unsigned int cell_len = 0;
  unsigned int ii;

  // read array size
  unsigned int size = deserialize_unsigned_int(buffer, read_len);
  length += read_len;

  // deserilize array
  for (ii = 0; ii < size; ii++) {
    int value = deserialize_int(buffer + length, cell_len);
    array.push_back(value);
    length += cell_len;
  }
  return array;
}

/**
 * Deserialize std::vector<long> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<long> deserialize_array_long(const unsigned char *buffer, unsigned int &length) {
  std::vector<long> array;
  unsigned int read_len = 0;
  length = 0;
  unsigned int cell_len = 0;
  unsigned int ii;

  // read array size
  unsigned int size = deserialize_unsigned_int(buffer, read_len);
  length += read_len;

  // deserilize array
  for (ii = 0; ii < size; ii++) {
    long value = deserialize_long(buffer + length, cell_len);
    array.push_back(value);
    length += cell_len;
  }
  return array;
}

/**
 * Deserialize std::vector<float> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<float> deserialize_array_float(const unsigned char *buffer, unsigned int &length) {
  std::vector<float> array;
  unsigned int read_len = 0;
  length = 0;
  unsigned int cell_len = 0;
  unsigned int ii;

  // read array size
  unsigned int size = deserialize_unsigned_int(buffer, read_len);
  length += read_len;

  // deserilize array
  for (ii = 0; ii < size; ii++) {
    float value = deserialize_float(buffer + length, cell_len);
    array.push_back(value);
    length += cell_len;
  }
  return array;
}

/**
 * Deserialize std::vector<double> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<double> deserialize_array_double(const unsigned char *buffer, unsigned int &length) {
  std::vector<double> array;
  unsigned int read_len = 0;
  length = 0;
  unsigned int cell_len = 0;
  unsigned int ii;

  // read array size
  unsigned int size = deserialize_unsigned_int(buffer, read_len);
  length += read_len;

  // deserilize array
  for (ii = 0; ii < size; ii++) {
    double value = deserialize_double(buffer + length, cell_len);
    array.push_back(value);
    length += cell_len;
  }
  return array;
}

/**
 * Deserialize std::vector<std::string> data.
 * @param buffer Data to be deserialized
 * @param length Data length
 * @return Deserialized data
 */
std::vector<std::string> deserialize_array_string(const unsigned char *buffer, unsigned int &length) {
  std::vector<std::string> array;
  unsigned int read_len = 0;
  length = 0;
  unsigned int cell_len = 0;
  unsigned int ii;

  // read array size
  unsigned int size = deserialize_unsigned_int(buffer, read_len);
  length += read_len;

  // deserilize array
  for (ii = 0; ii < size; ii++) {
    std::string value = deserialize_string(buffer + length, cell_len);
    array.push_back(value);
    length += cell_len;
  }
  return array;
}

/**
 * Serialize float data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_float(float number, unsigned char *buffer) {
  unsigned char *data = reinterpret_cast<unsigned char*>(&number);
  unsigned int ii;
  for (ii = 0; ii < sizeof(float); ii++) {
    buffer[ii] = data[ii];
  }
  return sizeof(float);
}

/**
 * Serialize double data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_double(double number, unsigned char *buffer) {
  unsigned char *data = reinterpret_cast<unsigned char*>(&number);
  unsigned int ii;
  for (ii = 0; ii < sizeof(double); ii++) {
    buffer[ii] = data[ii];
  }
  return sizeof(double);
}

/**
 * Serialize unsigned char data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_unsigned_char(unsigned char number, unsigned char *buffer) {
  unsigned char *data = reinterpret_cast<unsigned char*>(&number);
  unsigned int ii;
  for (ii = 0; ii < sizeof(unsigned char); ii++) {
    buffer[ii] = data[ii];
  }
  return sizeof(unsigned char);
}

/**
 * Serialize unsigned short data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_unsigned_short(unsigned short number, unsigned char *buffer) {
  unsigned char *data = reinterpret_cast<unsigned char*>(&number);
  unsigned int ii;
  for (ii = 0; ii < sizeof(unsigned short); ii++) {
    buffer[ii] = data[ii];
  }
  return sizeof(unsigned short);
}

/**
 * Serialize unsigned int data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_unsigned_int(unsigned int number, unsigned char *buffer) {
  unsigned char *data = reinterpret_cast<unsigned char*>(&number);
  unsigned int ii;
  for (ii = 0; ii < sizeof(unsigned int); ii++) {
    buffer[ii] = data[ii];
  }
  return sizeof(unsigned int);
}

/**
 * Serialize unsigned long data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_unsigned_long(unsigned long number, unsigned char *buffer) {
  unsigned char *data = reinterpret_cast<unsigned char*>(&number);
  unsigned int ii;
  for (ii = 0; ii < sizeof(unsigned long); ii++) {
    buffer[ii] = data[ii];
  }
  return sizeof(unsigned long);
}

/**
 * Serialize char data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_char(char number, unsigned char *buffer) {
  unsigned char *data = reinterpret_cast<unsigned char*>(&number);
  unsigned int ii;
  for (ii = 0; ii < sizeof(char); ii++) {
    buffer[ii] = data[ii];
  }
  return sizeof(char);
}

/**
 * Serialize short data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_short(short number, unsigned char *buffer) {
  unsigned char *data = reinterpret_cast<unsigned char*>(&number);
  unsigned int ii;
  for (ii = 0; ii < sizeof(short); ii++) {
    buffer[ii] = data[ii];
  }
  return sizeof(short);
}

/**
 * Serialize int data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_int(int number, unsigned char *buffer) {
  unsigned char *data = reinterpret_cast<unsigned char*>(&number);
  unsigned int ii;
  for (ii = 0; ii < sizeof(int); ii++) {
    buffer[ii] = data[ii];
  }
  return sizeof(int);
}

/**
 * Serialize long data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_long(long number, unsigned char *buffer) {
  unsigned char *data = reinterpret_cast<unsigned char*>(&number);
  unsigned int ii;
  for (ii = 0; ii < sizeof(long); ii++) {
    buffer[ii] = data[ii];
  }
  return sizeof(long);
}

/**
 * Serialize std::string data.
 * @param number Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_string(const std::string &str, unsigned char *buffer) {
  std::string redeclaration_str = std::string(str.c_str());
  unsigned int str_len = redeclaration_str.length() + 1;
  unsigned int write_len = serialize_unsigned_int(str_len, buffer);
  unsigned int ii;

  char* data_str = const_cast<char*>(str.c_str());
  unsigned char *str_buffer = reinterpret_cast<unsigned char*>(data_str);
  for (ii = 0; ii < str_len; ii++) {
    buffer[write_len + ii] = str_buffer[ii];
  }
  write_len += str_len;
  return write_len;
}

/**
 * Serialize std::vector<unsigned char> data.
 * @param array  Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_array_unsigned_char(const std::vector<unsigned char> &array, unsigned char *buffer) {
  unsigned int size = array.size();
  unsigned int write_len = serialize_unsigned_int(size, buffer);
  unsigned int ii;

  for (ii = 0; ii < size; ii++) {
    unsigned char value = (unsigned char)array[ii];
    write_len += serialize_unsigned_char(value, buffer + write_len);
  }
  return write_len;
}

/**
 * Serialize std::vector<unsigned short> data.
 * @param array  Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_array_unsigned_short(const std::vector<unsigned short> &array, unsigned char *buffer) {
  unsigned int size = array.size();
  unsigned int write_len = serialize_unsigned_int(size, buffer);
  unsigned int ii;

  for (ii = 0; ii < size; ii++) {
    unsigned short value = (unsigned short)array[ii];
    write_len += serialize_unsigned_short(value, buffer + write_len);
  }
  return write_len;
}

/**
 * Serialize std::vector<unsigned int> data.
 * @param array  Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_array_unsigned_int(const std::vector<unsigned int> &array, unsigned char *buffer) {
  unsigned int size = array.size();
  unsigned int write_len = serialize_unsigned_int(size, buffer);
  unsigned int ii;

  for (ii = 0; ii < size; ii++) {
    unsigned int value = (unsigned int)array[ii];
    write_len += serialize_unsigned_int(value, buffer + write_len);
  }
  return write_len;
}

/**
 * Serialize std::vector<unsigned long> data.
 * @param array  Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_array_unsigned_long(const std::vector<unsigned long> &array, unsigned char *buffer) {
  unsigned int size = array.size();
  unsigned int write_len = serialize_unsigned_int(size, buffer);
  unsigned int ii;

  for (ii = 0; ii < size; ii++) {
    unsigned long value = (unsigned long)array[ii];
    write_len += serialize_unsigned_long(value, buffer + write_len);
  }
  return write_len;
}

/**
 * Serialize std::vector<char> data.
 * @param array  Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_array_char(const std::vector<char> &array, unsigned char *buffer) {
  unsigned int size = array.size();
  unsigned int write_len = serialize_unsigned_int(size, buffer);
  unsigned int ii;

  for (ii = 0; ii < size; ii++) {
    char value = (char)array[ii];
    write_len += serialize_char(value, buffer + write_len);
  }
  return write_len;
}

/**
 * Serialize std::vector<short> data.
 * @param array  Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_array_short(const std::vector<short> &array, unsigned char *buffer) {
  unsigned int size = array.size();
  unsigned int write_len = serialize_unsigned_int(size, buffer);
  unsigned int ii;

  for (ii = 0; ii < size; ii++) {
    short value = (short)array[ii];
    write_len += serialize_short(value, buffer + write_len);
  }
  return write_len;
}

/**
 * Serialize std::vector<int> data.
 * @param array  Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_array_int(const std::vector<int> &array, unsigned char *buffer) {
  unsigned int size = array.size();
  unsigned int write_len = serialize_unsigned_int(size, buffer);
  unsigned int ii;

  for (ii = 0; ii < size; ii++) {
    int value = (int)array[ii];
    write_len += serialize_int(value, buffer + write_len);
  }
  return write_len;
}

/**
 * Serialize std::vector<long> data.
 * @param array  Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_array_long(const std::vector<long> &array, unsigned char *buffer) {
  unsigned int size = array.size();
  unsigned int write_len = serialize_unsigned_int(size, buffer);
  unsigned int ii;

  for (ii = 0; ii < size; ii++) {
    long value = (long)array[ii];
    write_len += serialize_long(value, buffer + write_len);
  }
  return write_len;
}

/**
 * Serialize std::vector<float> data.
 * @param array  Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_array_float(const std::vector<float> &array, unsigned char *buffer) {
  unsigned int size = array.size();
  unsigned int write_len = serialize_unsigned_int(size, buffer);
  unsigned int ii;

  for (ii = 0; ii < size; ii++) {
    float value = (float)array[ii];
    write_len += serialize_float(value, buffer + write_len);
  }
  return write_len;
}

/**
 * Serialize std::vector<double> data.
 * @param array  Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_array_double(const std::vector<double> &array, unsigned char *buffer) {
  unsigned int size = array.size();
  unsigned int write_len = serialize_unsigned_int(size, buffer);
  unsigned int ii;

  for (ii = 0; ii < size; ii++) {
    double value = (double)array[ii];
    write_len += serialize_double(value, buffer + write_len);
  }
  return write_len;
}

/**
 * Serialize std::vector<std::string> data.
 * @param array  Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
unsigned int serialize_array_string(const std::vector<std::string> &array, unsigned char *buffer) {
  unsigned int size = array.size();
  unsigned int write_len = serialize_unsigned_int(size, buffer);
  unsigned int ii;

  for (ii = 0; ii < size; ii++) {
    std::string value = (std::string)array[ii];
    write_len += serialize_string(value, buffer + write_len);
  }
  return write_len;
}
