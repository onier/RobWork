#ifndef RTDE_RTDE_UTILITY_H
#define RTDE_RTDE_UTILITY_H

#include <rtde_export.h>
#include <cstdint>
#include <vector>
#include <iomanip>

class RTDEUtility
{
 public:

  static inline std::vector<char> packInt32(int32_t int32)
  {
    std::vector<char> result;
    result.push_back(int32 >> 24);
    result.push_back(int32 >> 16);
    result.push_back(int32 >>  8);
    result.push_back(int32);
    return result;
  }

  static inline std::vector<char> packVectorNInt32(std::vector<int32_t> vector_n_int32)
  {
    std::vector<char> result;
    for (auto i : vector_n_int32)
    {
      result.push_back(i >> 24);
      result.push_back(i >> 16);
      result.push_back(i >>  8);
      result.push_back(i);
    }

    return result;
  }

  static inline std::vector<char> packVectorNd(std::vector<double> vector_nd)
  {
    std::vector<char> output;

    for (auto d : vector_nd)
    {
      union temp
      {
        double value;
        char c[8];
      } in{}, out{};

      in.value = d;
      out.c[0] = in.c[7];
      out.c[1] = in.c[6];
      out.c[2] = in.c[5];
      out.c[3] = in.c[4];
      out.c[4] = in.c[3];
      out.c[5] = in.c[2];
      out.c[6] = in.c[1];
      out.c[7] = in.c[0];

      for (auto const& character : out.c)
        output.push_back(character);
    }

    return output;
  }

  static inline std::vector<double> unpackVector3d(const std::vector<char>& data, uint32_t& message_offset)
  {
    std::vector<double> vector_3d;
    for (unsigned int i = 0; i < 3; i++)
    {
      double d = getDouble(data, message_offset);
      vector_3d.push_back(d);
    }
    return vector_3d;
  }

  static inline std::vector<double> unpackVector6d(const std::vector<char>& data, uint32_t& message_offset)
  {
    std::vector<double> vector_6d;
    for (unsigned int i = 0; i < 6; i++)
    {
      double d = getDouble(data, message_offset);
      vector_6d.push_back(d);
    }
    return vector_6d;
  }

  static inline std::vector<int32_t> unpackVector6Int32(const std::vector<char>& data, uint32_t& message_offset)
  {
    std::vector<int32_t> vector_6_int32;
    for (unsigned int i = 0; i < 6; i++)
    {
      int32_t int32_value = getInt32(data, message_offset);
      vector_6_int32.push_back(int32_value);
    }
    return vector_6_int32;
  }

  static inline double getDouble(const std::vector<char>& data, uint32_t& message_offset)
  {
    double output;

    ((char*)(&output))[7] = data[message_offset];
    ((char*)(&output))[6] = data[message_offset + 1];
    ((char*)(&output))[5] = data[message_offset + 2];
    ((char*)(&output))[4] = data[message_offset + 3];
    ((char*)(&output))[3] = data[message_offset + 4];
    ((char*)(&output))[2] = data[message_offset + 5];
    ((char*)(&output))[1] = data[message_offset + 6];
    ((char*)(&output))[0] = data[message_offset + 7];

    message_offset += 8;
    return output;
  }

  static inline uint32_t getUInt32(const std::vector<char>& data, uint32_t& message_offset)
  {
    uint32_t output = 0;
    ((char*)(&output))[3] = data[message_offset];
    ((char*)(&output))[2] = data[message_offset + 1];
    ((char*)(&output))[1] = data[message_offset + 2];
    ((char*)(&output))[0] = data[message_offset + 3];
    message_offset += 4;

    return output;
  }

  static inline uint16_t getUInt16(const std::vector<char>& data, uint32_t& message_offset)
  {
    uint16_t output = 0;
    ((char*)(&output))[1] = data[message_offset + 0];
    ((char*)(&output))[0] = data[message_offset + 1];
    message_offset += 2;

    return output;
  }

  static inline uint32_t getInt32(const std::vector<char>& data, uint32_t& message_offset)
  {
    uint32_t output = 0;
    ((char*)(&output))[3] = data[message_offset];
    ((char*)(&output))[2] = data[message_offset + 1];
    ((char*)(&output))[1] = data[message_offset + 2];
    ((char*)(&output))[0] = data[message_offset + 3];
    message_offset += 4;

    return -(output & 0x80000000) + (output & 0x7fffffff);
  }

  static inline uint64_t getUInt64(const std::vector<char>& data, uint32_t& message_offset)
  {
    uint64_t output;

    ((char*)(&output))[7] = data[message_offset];
    ((char*)(&output))[6] = data[message_offset + 1];
    ((char*)(&output))[5] = data[message_offset + 2];
    ((char*)(&output))[4] = data[message_offset + 3];
    ((char*)(&output))[3] = data[message_offset + 4];
    ((char*)(&output))[2] = data[message_offset + 5];
    ((char*)(&output))[1] = data[message_offset + 6];
    ((char*)(&output))[0] = data[message_offset + 7];

    message_offset += 8;
    return output;
  }

  static inline unsigned char getUChar(const std::vector<char>& data, uint32_t& message_offset)
  {
    unsigned char output = data[message_offset];
    message_offset += 1;
    return output;
  }

  static inline std::string double2hexstr(double x)
  {
    union
    {
      long long i;
      double d;
    } value;

    value.d = x;

    std::ostringstream buf;
    buf << std::hex << std::setw(6) << value.i;
    return buf.str();
  }

  static inline std::vector<char> hexToBytes(const std::string& hex)
  {
    std::vector<char> bytes;

    for (unsigned int i = 0; i < hex.length(); i += 2)
    {
      std::string byteString = hex.substr(i, 2);
      char byte = (char)strtol(byteString.c_str(), nullptr, 16);
      bytes.push_back(byte);
    }

    return bytes;
  }

  static inline std::ostream& hexDump(std::ostream& o, char const* p, std::size_t size)
  {
    o << std::hex << std::setw(2) << std::setfill('0');
    while (size--)
      o << (static_cast<unsigned int>(*p++) & 0xff) << ' ';
    return o;
  }

  static inline std::vector<std::string> split(const std::string& s, char delimiter)
  {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
      tokens.push_back(token);
    }
    return tokens;
  }
};

#endif  // RTDE_RTDE_UTILITY_H
