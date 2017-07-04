/**
 * @file md5.h
 * @The header file of md5.
 * @author Jiewei Wei
 * @mail weijieweijerry@163.com
 * @github https://github.com/JieweiWei
 * @data Oct 19 2014
 *
 */

#ifndef CPP_CORE_MD5_H
#define CPP_CORE_MD5_H

#include <string>
#include <cstring>

namespace ros
{

namespace md5
{

/* Parameters of MD5. */
static const int s11 = 7;
static const int s12 = 12;
static const int s13 = 17;
static const int s14 = 22;
static const int s21 = 5;
static const int s22 = 9;
static const int s23 = 14;
static const int s24 = 20;
static const int s31 = 4;
static const int s32 = 11;
static const int s33 = 16;
static const int s34 = 23;
static const int s41 = 6;
static const int s42 = 10;
static const int s43 = 15;
static const int s44 = 21;

/**
 * @brief Basic MD5 functions.
 * @param there uint32_t.
 * @return one uint32_t.
 */
#define ROS_MD5_F(x, y, z) (((x) & (y)) | ((~x) & (z)))
#define ROS_MD5_G(x, y, z) (((x) & (z)) | ((y) & (~z)))
#define ROS_MD5_H(x, y, z) ((x) ^ (y) ^ (z))
#define ROS_MD5_I(x, y, z) ((y) ^ ((x) | (~z)))

/**
 * @brief Rotate Left.
 * @param {num} the raw number.
 * @param {n} rotate left n.
 * @return the number after rotated left.
 */
#define ROS_MD5_ROTATELEFT(num, n) (((num) << (n)) | ((num) >> (32-(n))))

/**
 * @brief Transformations for rounds 1, 2, 3, and 4.
 */
#define ROS_MD5_FF(a, b, c, d, x, s, ac)        \
{                                               \
  (a) += ROS_MD5_F ((b), (c), (d)) + (x) + ac;  \
  (a) = ROS_MD5_ROTATELEFT ((a), (s));          \
  (a) += (b);                                   \
}

#define ROS_MD5_GG(a, b, c, d, x, s, ac)        \
{                                               \
  (a) += ROS_MD5_G ((b), (c), (d)) + (x) + ac;  \
  (a) = ROS_MD5_ROTATELEFT ((a), (s));          \
  (a) += (b);                                   \
}

#define ROS_MD5_HH(a, b, c, d, x, s, ac)        \
{                                               \
  (a) += ROS_MD5_H ((b), (c), (d)) + (x) + ac;  \
  (a) = ROS_MD5_ROTATELEFT ((a), (s));          \
  (a) += (b);                                   \
}

#define ROS_MD5_II(a, b, c, d, x, s, ac)        \
{                                               \
  (a) += ROS_MD5_I ((b), (c), (d)) + (x) + ac;  \
  (a) = ROS_MD5_ROTATELEFT ((a), (s));          \
  (a) += (b);                                   \
}

using std::string;

typedef unsigned char byte;

static const byte PADDING[64] = { 0x80 };
static const char HEX_NUMBERS[16] = {'0', '1', '2', '3',
                                     '4', '5', '6', '7',
                                     '8', '9', 'a', 'b',
                                     'c', 'd', 'e', 'f'};

class MD5 
{
public:
  MD5(const string& message) : finished_(false)
  {
    count_[0] = 0;
    count_[1] = 0;
    state_[0] = 0x67452301;
    state_[1] = 0xefcdab89;
    state_[2] = 0x98badcfe;
    state_[3] = 0x10325476;
    /* Initialization the object according to message. */
    init((const byte*)message.c_str(), message.length());
  }

  /* Generate md5 digest_. */
  const byte* getDigest() 
  {
    if (!finished_) 
    {
      finished_ = true;

      byte bits[8];
      uint32_t oldState[4];
      uint32_t oldCount[2];
      uint32_t index = 0;
      uint32_t padLen = 0;

      /* Save current state_ and count_. */
      memcpy(oldState, state_, 16);
      memcpy(oldCount, count_, 8);

      /* Save number of bits */
      encode(count_, bits, 8);

      /* Pad out to 56 mod 64. */
      index = static_cast<uint32_t>((count_[0] >> 3) & 0x3f);
      padLen = (index < 56) ? (56 - index) : (120 - index);
      init(PADDING, padLen);

      /* Append length (before padding) */
      init(bits, 8);

      /* Store state_ in digest_ */
      encode(state_, digest_, 16);

      /* Restore current state_ and count_. */
      memcpy(state_, oldState, 16);
      memcpy(count_, oldCount, 8);
    }
    return digest_;
  }

  /* Convert digest_ to string value */
  string toStr()
  {
    const byte* digest = getDigest();
    string str;
    str.reserve(16 << 1);
    for (size_t i = 0; i < 16; ++i) {
      int t = digest[i];
      int a = t / 16;
      int b = t % 16;
      str.append(1, HEX_NUMBERS[a]);
      str.append(1, HEX_NUMBERS[b]);
    }
    return str;
  }

private:
  /* Initialization the md5 object, processing another message block,
   * and updating the context.*/
  void init(const byte* input, size_t len)
  {
    uint32_t i = 0;
    uint32_t index = 0;
    uint32_t partLen = 0;
    uint32_t length = static_cast<uint32_t>(len);

    finished_ = false;

    /* Compute number of bytes mod 64 */
    index = static_cast<uint32_t>((count_[0] >> 3) & 0x3f);

    /* update number of bits */
    if ((count_[0] += (length << 3)) < (length << 3)) {
      ++count_[1];
    }
    count_[1] += length >> 29;

    partLen = 64 - index;

    /* transform as many times as possible. */
    if (length >= partLen) 
    {
      memcpy(&buffer_[index], input, partLen);
      transform(buffer_);

      for (i = partLen; i + 63 < length; i += 64) 
      {
        transform(&input[i]);
      }
      index = 0;
    } 
    else 
    {
      i = 0;
    }
    /* Buffer remaining input */
    memcpy(&buffer_[index], &input[i], length - i);
  }

  /* MD5 basic transformation. Transforms state_ based on block. */
  void transform(const byte block[64])
  {
    uint32_t a = state_[0];
    uint32_t b = state_[1];
    uint32_t c = state_[2];
    uint32_t d = state_[3];
    uint32_t x[16];

    decode(block, x, 64);

    /* Round 1 */
    ROS_MD5_FF (a, b, c, d, x[ 0], s11, 0xd76aa478);
    ROS_MD5_FF (d, a, b, c, x[ 1], s12, 0xe8c7b756);
    ROS_MD5_FF (c, d, a, b, x[ 2], s13, 0x242070db);
    ROS_MD5_FF (b, c, d, a, x[ 3], s14, 0xc1bdceee);
    ROS_MD5_FF (a, b, c, d, x[ 4], s11, 0xf57c0faf);
    ROS_MD5_FF (d, a, b, c, x[ 5], s12, 0x4787c62a);
    ROS_MD5_FF (c, d, a, b, x[ 6], s13, 0xa8304613);
    ROS_MD5_FF (b, c, d, a, x[ 7], s14, 0xfd469501);
    ROS_MD5_FF (a, b, c, d, x[ 8], s11, 0x698098d8);
    ROS_MD5_FF (d, a, b, c, x[ 9], s12, 0x8b44f7af);
    ROS_MD5_FF (c, d, a, b, x[10], s13, 0xffff5bb1);
    ROS_MD5_FF (b, c, d, a, x[11], s14, 0x895cd7be);
    ROS_MD5_FF (a, b, c, d, x[12], s11, 0x6b901122);
    ROS_MD5_FF (d, a, b, c, x[13], s12, 0xfd987193);
    ROS_MD5_FF (c, d, a, b, x[14], s13, 0xa679438e);
    ROS_MD5_FF (b, c, d, a, x[15], s14, 0x49b40821);

    /* Round 2 */
    ROS_MD5_GG (a, b, c, d, x[ 1], s21, 0xf61e2562);
    ROS_MD5_GG (d, a, b, c, x[ 6], s22, 0xc040b340);
    ROS_MD5_GG (c, d, a, b, x[11], s23, 0x265e5a51);
    ROS_MD5_GG (b, c, d, a, x[ 0], s24, 0xe9b6c7aa);
    ROS_MD5_GG (a, b, c, d, x[ 5], s21, 0xd62f105d);
    ROS_MD5_GG (d, a, b, c, x[10], s22,  0x2441453);
    ROS_MD5_GG (c, d, a, b, x[15], s23, 0xd8a1e681);
    ROS_MD5_GG (b, c, d, a, x[ 4], s24, 0xe7d3fbc8);
    ROS_MD5_GG (a, b, c, d, x[ 9], s21, 0x21e1cde6);
    ROS_MD5_GG (d, a, b, c, x[14], s22, 0xc33707d6);
    ROS_MD5_GG (c, d, a, b, x[ 3], s23, 0xf4d50d87);
    ROS_MD5_GG (b, c, d, a, x[ 8], s24, 0x455a14ed);
    ROS_MD5_GG (a, b, c, d, x[13], s21, 0xa9e3e905);
    ROS_MD5_GG (d, a, b, c, x[ 2], s22, 0xfcefa3f8);
    ROS_MD5_GG (c, d, a, b, x[ 7], s23, 0x676f02d9);
    ROS_MD5_GG (b, c, d, a, x[12], s24, 0x8d2a4c8a);

    /* Round 3 */
    ROS_MD5_HH (a, b, c, d, x[ 5], s31, 0xfffa3942);
    ROS_MD5_HH (d, a, b, c, x[ 8], s32, 0x8771f681);
    ROS_MD5_HH (c, d, a, b, x[11], s33, 0x6d9d6122);
    ROS_MD5_HH (b, c, d, a, x[14], s34, 0xfde5380c);
    ROS_MD5_HH (a, b, c, d, x[ 1], s31, 0xa4beea44);
    ROS_MD5_HH (d, a, b, c, x[ 4], s32, 0x4bdecfa9);
    ROS_MD5_HH (c, d, a, b, x[ 7], s33, 0xf6bb4b60);
    ROS_MD5_HH (b, c, d, a, x[10], s34, 0xbebfbc70);
    ROS_MD5_HH (a, b, c, d, x[13], s31, 0x289b7ec6);
    ROS_MD5_HH (d, a, b, c, x[ 0], s32, 0xeaa127fa);
    ROS_MD5_HH (c, d, a, b, x[ 3], s33, 0xd4ef3085);
    ROS_MD5_HH (b, c, d, a, x[ 6], s34,  0x4881d05);
    ROS_MD5_HH (a, b, c, d, x[ 9], s31, 0xd9d4d039);
    ROS_MD5_HH (d, a, b, c, x[12], s32, 0xe6db99e5);
    ROS_MD5_HH (c, d, a, b, x[15], s33, 0x1fa27cf8);
    ROS_MD5_HH (b, c, d, a, x[ 2], s34, 0xc4ac5665);

    /* Round 4 */
    ROS_MD5_II (a, b, c, d, x[ 0], s41, 0xf4292244);
    ROS_MD5_II (d, a, b, c, x[ 7], s42, 0x432aff97);
    ROS_MD5_II (c, d, a, b, x[14], s43, 0xab9423a7);
    ROS_MD5_II (b, c, d, a, x[ 5], s44, 0xfc93a039);
    ROS_MD5_II (a, b, c, d, x[12], s41, 0x655b59c3);
    ROS_MD5_II (d, a, b, c, x[ 3], s42, 0x8f0ccc92);
    ROS_MD5_II (c, d, a, b, x[10], s43, 0xffeff47d);
    ROS_MD5_II (b, c, d, a, x[ 1], s44, 0x85845dd1);
    ROS_MD5_II (a, b, c, d, x[ 8], s41, 0x6fa87e4f);
    ROS_MD5_II (d, a, b, c, x[15], s42, 0xfe2ce6e0);
    ROS_MD5_II (c, d, a, b, x[ 6], s43, 0xa3014314);
    ROS_MD5_II (b, c, d, a, x[13], s44, 0x4e0811a1);
    ROS_MD5_II (a, b, c, d, x[ 4], s41, 0xf7537e82);
    ROS_MD5_II (d, a, b, c, x[11], s42, 0xbd3af235);
    ROS_MD5_II (c, d, a, b, x[ 2], s43, 0x2ad7d2bb);
    ROS_MD5_II (b, c, d, a, x[ 9], s44, 0xeb86d391);

    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
  }

  /* Encodes input (usigned long) into output (byte). */
  void encode(const uint32_t* input, byte* output, size_t length)
  {
    for (size_t i = 0, j = 0; j < length; ++i, j += 4) 
    {
      output[j]= (byte)(input[i] & 0xff);
      output[j + 1] = (byte)((input[i] >> 8) & 0xff);
      output[j + 2] = (byte)((input[i] >> 16) & 0xff);
      output[j + 3] = (byte)((input[i] >> 24) & 0xff);
    }
  }

  /* Decodes input (byte) into output (usigned long). */
  void decode(const byte* input, uint32_t* output, size_t length)
  {
    for (size_t i = 0, j = 0; j < length; ++i, j += 4) 
    {
      output[i] = (uint32_t)input[j] | 
                  (uint32_t)input[j + 1] << 8 |
                  (uint32_t)input[j + 2] << 16 | 
                  (uint32_t)input[j + 3] << 24;
    }
  }

private:
  /* Flag for mark whether calculate finished_. */
  bool finished_;

	/* state_ (ABCD). */
  uint32_t state_[4];

  /* number of bits, low-order word first. */
  uint32_t count_[2];

  /* input buffer_. */
  byte buffer_[64];

  /* message digest_. */
  byte digest_[16];

};

} // namespace md5

} // namespace ros

#endif // CPP_CORE_MD5_H
