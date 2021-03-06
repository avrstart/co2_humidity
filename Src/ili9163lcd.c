/************************************************************************
	ili9163lcd.c

    ILI9163 128x128 LCD library
    Copyright (C) 2012 Simon Inns

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

	Email: simon.inns@gmail.com

************************************************************************/
#include "ili9163lcd.h"

// 5x8 Font data
// This font was created by Philippe Lucidarme and is available
// from http://piclist.com/techref/datafile/charset/8x6.htm
//
// Note: This table is arranged according to ASCII codes 0 to 255
const uint8_t font8x5[][6] = {
{0x00,0x00,0x00,0x00,0x00,0x00},   //   0x00 0
{0x00,0x64,0x18,0x04,0x64,0x18},   //   0x01 1
{0x00,0x3c,0x40,0x40,0x20,0x7c},   //   0x02 2
{0x00,0x0c,0x30,0x40,0x30,0x0c},   //   0x03 3
{0x00,0x3c,0x40,0x30,0x40,0x3c},   //   0x04 4
{0x00,0x00,0x3e,0x1c,0x08,0x00},   //   0x05 5
{0x00,0x04,0x1e,0x1f,0x1e,0x04},   //   0x06 6
{0x00,0x10,0x3c,0x7c,0x3c,0x10},   //   0x07 7
{0x00,0x20,0x40,0x3e,0x01,0x02},   //   0x08 8
{0x00,0x22,0x14,0x08,0x14,0x22},   //   0x09 9
{0x00,0x00,0x38,0x28,0x38,0x00},   //   0x0a 10
{0x00,0x00,0x10,0x38,0x10,0x00},   //  0x0b 11
{0x00,0x00,0x00,0x10,0x00,0x00},   //  0x0c 12
{0x00,0x08,0x78,0x08,0x00,0x00},   //   0x0d 13
{0x00,0x00,0x15,0x15,0x0a,0x00},   //   0x0e 14
{0x00,0x7f,0x7f,0x09,0x09,0x01},   //   0x0f 15
{0x00,0x10,0x20,0x7f,0x01,0x01},   //   0x10 16
{0x00,0x04,0x04,0x00,0x01,0x1f},   //   0x11 17
{0x00,0x00,0x19,0x15,0x12,0x00},   //   0x12 18
{0x00,0x40,0x60,0x50,0x48,0x44},   //   0x13 19
{0x00,0x06,0x09,0x09,0x06,0x00},   //   0x14 20
{0x00,0x0f,0x02,0x01,0x01,0x00},   //   0x15 21
{0x00,0x00,0x01,0x1f,0x01,0x00},   //   0x16 22
{0x00,0x44,0x44,0x4a,0x4a,0x51},   //   0x17 23
{0x00,0x14,0x74,0x1c,0x17,0x14},   //   0x18 24
{0x00,0x51,0x4a,0x4a,0x44,0x44},   //   0x19 25
{0x00,0x00,0x00,0x04,0x04,0x04},   //   0x1a 26
{0x00,0x00,0x7c,0x54,0x54,0x44},   //   0x1b 27
{0x00,0x08,0x08,0x2a,0x1c,0x08},   //   0x1c 28
{0x00,0x7c,0x00,0x7c,0x44,0x7c},   //   0x1d 29
{0x00,0x04,0x02,0x7f,0x02,0x04},   //   0x1e 30
{0x00,0x10,0x20,0x7f,0x20,0x10},   //   0x1f 31
{0x00,0x00,0x00,0x00,0x00,0x00},   //   0x20 32
{0x00,0x00,0x00,0x6f,0x00,0x00},   // ! 0x21 33
{0x00,0x00,0x07,0x00,0x07,0x00},   // " 0x22 34
{0x00,0x14,0x7f,0x14,0x7f,0x14},   // # 0x23 35
{0x00,0x00,0x07,0x04,0x1e,0x00},   // $ 0x24 36
{0x00,0x23,0x13,0x08,0x64,0x62},   // % 0x25 37
{0x00,0x36,0x49,0x56,0x20,0x50},   // & 0x26 38
{0x00,0x00,0x00,0x07,0x00,0x00},   // ' 0x27 39
{0x00,0x00,0x1c,0x22,0x41,0x00},   // ( 0x28 40
{0x00,0x00,0x41,0x22,0x1c,0x00},   // ) 0x29 41
{0x00,0x14,0x08,0x3e,0x08,0x14},   // * 0x2a 42
{0x00,0x08,0x08,0x3e,0x08,0x08},   // + 0x2b 43
{0x00,0x00,0x50,0x30,0x00,0x00},   // , 0x2c 44
{0x00,0x08,0x08,0x08,0x08,0x08},   // - 0x2d 45
{0x00,0x00,0x60,0x60,0x00,0x00},   // . 0x2e 46
{0x00,0x20,0x10,0x08,0x04,0x02},   // / 0x2f 47
{0x00,0x3e,0x51,0x49,0x45,0x3e},   // 0 0x30 48
{0x00,0x00,0x42,0x7f,0x40,0x00},   // 1 0x31 49
{0x00,0x42,0x61,0x51,0x49,0x46},   // 2 0x32 50
{0x00,0x21,0x41,0x45,0x4b,0x31},   // 3 0x33 51
{0x00,0x18,0x14,0x12,0x7f,0x10},   // 4 0x34 52
{0x00,0x27,0x45,0x45,0x45,0x39},   // 5 0x35 53
{0x00,0x3c,0x4a,0x49,0x49,0x30},   // 6 0x36 54
{0x00,0x01,0x71,0x09,0x05,0x03},   // 7 0x37 55
{0x00,0x36,0x49,0x49,0x49,0x36},   // 8 0x38 56
{0x00,0x06,0x49,0x49,0x29,0x1e},   // 9 0x39 57
{0x00,0x00,0x36,0x36,0x00,0x00},   // : 0x3a 58
{0x00,0x00,0x56,0x36,0x00,0x00},   // ; 0x3b 59
{0x00,0x08,0x14,0x22,0x41,0x00},   // < 0x3c 60
{0x00,0x14,0x14,0x14,0x14,0x14},   // = 0x3d 61
{0x00,0x00,0x41,0x22,0x14,0x08},   // > 0x3e 62
{0x00,0x02,0x01,0x51,0x09,0x06},   // ? 0x3f 63
{0x00,0x3e,0x41,0x5d,0x49,0x4e},   // @ 0x40 64
{0x00,0x7e,0x09,0x09,0x09,0x7e},   // A 0x41 65
{0x00,0x7f,0x49,0x49,0x49,0x36},   // B 0x42 66
{0x00,0x3e,0x41,0x41,0x41,0x22},   // C 0x43 67
{0x00,0x7f,0x41,0x41,0x41,0x3e},   // D 0x44 68
{0x00,0x7f,0x49,0x49,0x49,0x41},   // E 0x45 69
{0x00,0x7f,0x09,0x09,0x09,0x01},   // F 0x46 70
{0x00,0x3e,0x41,0x49,0x49,0x7a},   // G 0x47 71
{0x00,0x7f,0x08,0x08,0x08,0x7f},   // H 0x48 72
{0x00,0x00,0x41,0x7f,0x41,0x00},   // I 0x49 73
{0x00,0x20,0x40,0x41,0x3f,0x01},   // J 0x4a 74
{0x00,0x7f,0x08,0x14,0x22,0x41},   // K 0x4b 75
{0x00,0x7f,0x40,0x40,0x40,0x40},   // L 0x4c 76
{0x00,0x7f,0x02,0x0c,0x02,0x7f},   // M 0x4d 77
{0x00,0x7f,0x04,0x08,0x10,0x7f},   // N 0x4e 78
{0x00,0x3e,0x41,0x41,0x41,0x3e},   // O 0x4f 79
{0x00,0x7f,0x09,0x09,0x09,0x06},   // P 0x50 80
{0x00,0x3e,0x41,0x51,0x21,0x5e},   // Q 0x51 81
{0x00,0x7f,0x09,0x19,0x29,0x46},   // R 0x52 82
{0x00,0x46,0x49,0x49,0x49,0x31},   // S 0x53 83
{0x00,0x01,0x01,0x7f,0x01,0x01},   // T 0x54 84
{0x00,0x3f,0x40,0x40,0x40,0x3f},   // U 0x55 85
{0x00,0x0f,0x30,0x40,0x30,0x0f},   // V 0x56 86
{0x00,0x3f,0x40,0x30,0x40,0x3f},   // W 0x57 87
{0x00,0x63,0x14,0x08,0x14,0x63},   // X 0x58 88
{0x00,0x07,0x08,0x70,0x08,0x07},   // Y 0x59 89
{0x00,0x61,0x51,0x49,0x45,0x43},   // Z 0x5a 90
{0x00,0x3c,0x4a,0x49,0x29,0x1e},   // [ 0x5b 91
{0x00,0x02,0x04,0x08,0x10,0x20},   // \ 0x5c 92
{0x00,0x00,0x41,0x7f,0x00,0x00},   // ] 0x5d 93
{0x00,0x04,0x02,0x01,0x02,0x04},   // ^ 0x5e 94
{0x00,0x40,0x40,0x40,0x40,0x40},   // _ 0x5f 95
{0x00,0x00,0x00,0x03,0x04,0x00},   // ` 0x60 96
{0x00,0x20,0x54,0x54,0x54,0x78},   // a 0x61 97
{0x00,0x7f,0x48,0x44,0x44,0x38},   // b 0x62 98
{0x00,0x38,0x44,0x44,0x44,0x20},   // c 0x63 99
{0x00,0x38,0x44,0x44,0x48,0x7f},   // d 0x64 100
{0x00,0x38,0x54,0x54,0x54,0x18},   // e 0x65 101
{0x00,0x08,0x7e,0x09,0x01,0x02},   // f 0x66 102
{0x00,0x0c,0x52,0x52,0x52,0x3e},   // g 0x67 103
{0x00,0x7f,0x08,0x04,0x04,0x78},   // h 0x68 104
{0x00,0x00,0x44,0x7d,0x40,0x00},   // i 0x69 105
{0x00,0x20,0x40,0x44,0x3d,0x00},   // j 0x6a 106
{0x00,0x00,0x7f,0x10,0x28,0x44},   // k 0x6b 107
{0x00,0x00,0x41,0x7f,0x40,0x00},   // l 0x6c 108
{0x00,0x7c,0x04,0x18,0x04,0x78},   // m 0x6d 109
{0x00,0x7c,0x08,0x04,0x04,0x78},   // n 0x6e 110
{0x00,0x38,0x44,0x44,0x44,0x38},   // o 0x6f 111
{0x00,0x7c,0x14,0x14,0x14,0x08},   // p 0x70 112
{0x00,0x08,0x14,0x14,0x18,0x7c},   // q 0x71 113
{0x00,0x7c,0x08,0x04,0x04,0x08},   // r 0x72 114
{0x00,0x48,0x54,0x54,0x54,0x20},   // s 0x73 115
{0x00,0x04,0x3f,0x44,0x40,0x20},   // t 0x74 116
{0x00,0x3c,0x40,0x40,0x20,0x7c},   // u 0x75 117
{0x00,0x1c,0x20,0x40,0x20,0x1c},   // v 0x76 118
{0x00,0x3c,0x40,0x30,0x40,0x3c},   // w 0x77 119
{0x00,0x44,0x28,0x10,0x28,0x44},   // x 0x78 120
{0x00,0x0c,0x50,0x50,0x50,0x3c},   // y 0x79 121
{0x00,0x44,0x64,0x54,0x4c,0x44},   // z 0x7a 122
{0x00,0x00,0x08,0x36,0x41,0x41},   // { 0x7b 123
{0x00,0x00,0x00,0x7f,0x00,0x00},   // | 0x7c 124
{0x00,0x41,0x41,0x36,0x08,0x00},   // } 0x7d 125
{0x00,0x04,0x02,0x04,0x08,0x04},   // ~ 0x7e 126
{0x00,0x7f,0x6b,0x6b,0x6b,0x7f},   //   0x7f 127
{0x00,0x00,0x7c,0x44,0x7c,0x00},   // ¬ 0x80 128
{0x00,0x00,0x08,0x7c,0x00,0x00},   // ? 0x81 129
{0x00,0x00,0x64,0x54,0x48,0x00},   // ‚ 0x82 130
{0x00,0x00,0x44,0x54,0x28,0x00},   // ѓ 0x83 131
{0x00,0x00,0x1c,0x10,0x78,0x00},   // „ 0x84 132
{0x00,0x00,0x5c,0x54,0x24,0x00},   // … 0x85 133
{0x00,0x00,0x78,0x54,0x74,0x00},   // † 0x86 134
{0x00,0x00,0x64,0x14,0x0c,0x00},   // ‡ 0x87 135
{0x00,0x00,0x7c,0x54,0x7c,0x00},   // € 0x88 136
{0x00,0x00,0x5c,0x54,0x3c,0x00},   // ‰ 0x89 137
{0x00,0x78,0x24,0x26,0x25,0x78},   // Љ 0x8a 138
{0x00,0x78,0x25,0x26,0x24,0x78},   // ‹ 0x8b 139
{0x00,0x70,0x2a,0x29,0x2a,0x70},   // Њ 0x8c 140
{0x00,0x78,0x25,0x24,0x25,0x78},   // ? 0x8d 141
{0x00,0x20,0x54,0x56,0x55,0x78},   // } 0x8e 142
{0x00,0x20,0x55,0x56,0x54,0x78},   // ? 0x8f 143
{0x00,0x20,0x56,0x55,0x56,0x78},   // ? 0x90 144
{0x00,0x20,0x55,0x54,0x55,0x78},   // ‘ 0x91 145
{0x00,0x7c,0x54,0x56,0x55,0x44},   // ’ 0x92 146
{0x00,0x7c,0x55,0x56,0x54,0x44},   // “ 0x93 147
{0x00,0x7c,0x56,0x55,0x56,0x44},   // ” 0x94 148
{0x00,0x7c,0x55,0x54,0x55,0x44},   // • 0x95 149
{0x00,0x38,0x54,0x56,0x55,0x18},   // – 0x96 150
{0x00,0x38,0x55,0x56,0x54,0x18},   // — 0x97 151
{0x00,0x38,0x56,0x55,0x56,0x18},   // � 0x98 152
{0x00,0x38,0x55,0x54,0x55,0x18},   // ™ 0x99 153
{0x00,0x00,0x44,0x7e,0x45,0x00},   // љ 0x9a 154
{0x00,0x00,0x45,0x7e,0x44,0x00},   // › 0x9b 155
{0x00,0x00,0x46,0x7d,0x46,0x00},   // њ 0x9c 156
{0x00,0x00,0x45,0x7c,0x45,0x00},   // ? 0x9d 157
{0x00,0x00,0x48,0x7a,0x41,0x00},   // ~ 0x9e 158
{0x00,0x00,0x49,0x7a,0x40,0x00},   // џ 0x9f 159
{0x00,0x00,0x4a,0x79,0x42,0x00},   //   0xa0 160
{0x00,0x00,0x49,0x78,0x41,0x00},   // Ў 0xa1 161
{0x00,0x38,0x44,0x46,0x45,0x38},   // ў 0xa2 162
{0x00,0x38,0x45,0x46,0x44,0x38},   // Ј 0xa3 163
{0x00,0x38,0x46,0x45,0x46,0x38},   // ¤ 0xa4 164
{0x00,0x38,0x45,0x44,0x45,0x38},   // Ґ 0xa5 165
{0x00,0x30,0x48,0x4a,0x49,0x30},   // ¦ 0xa6 166
{0x00,0x30,0x49,0x4a,0x48,0x30},   // § 0xa7 167
{0x00,0x30,0x4a,0x49,0x4a,0x30},   // Ё 0xa8 168
{0x00,0x30,0x49,0x48,0x49,0x30},   // © 0xa9 169
{0x00,0x3c,0x40,0x42,0x41,0x3c},   // Є 0xaa 170
{0x00,0x3c,0x41,0x42,0x40,0x3c},   // « 0xab 171
{0x00,0x3c,0x42,0x41,0x42,0x3c},   // ¬ 0xac 172
{0x00,0x3c,0x41,0x40,0x41,0x3c},   // ­  0xad 173
{0x00,0x3c,0x40,0x42,0x21,0x7c},   // ® 0xae 174
{0x00,0x3c,0x41,0x42,0x20,0x7c},   // Ї 0xaf 175
{0x00,0x38,0x42,0x41,0x22,0x78},   // ° 0xb0 176
{0x00,0x3c,0x41,0x40,0x21,0x7c},   // ± 0xb1 177
{0x00,0x4e,0x51,0x71,0x11,0x0a},   // І 0xb2 178
{0x00,0x58,0x64,0x64,0x24,0x10},   // і 0xb3 179
{0x00,0x7c,0x0a,0x11,0x22,0x7d},   // ґ 0xb4 180
{0x00,0x78,0x12,0x09,0x0a,0x71},   // µ 0xb5 181
{0x00,0x00,0x00,0x04,0x02,0x01},   // ¶ 0xb6 182
{0x00,0x01,0x02,0x04,0x00,0x00},   // · 0xb7 183
{0x00,0x00,0x02,0x00,0x02,0x00},   // ё 0xb8 184
{0x00,0x30,0x48,0x45,0x40,0x20},   // № 0xb9 185
{0x00,0x00,0x00,0x7b,0x00,0x00},   // є 0xba 186
{0x00,0x38,0x44,0x44,0x38,0x44},   // » 0xbb 187
{0x00,0x40,0x3e,0x49,0x49,0x36},   // ј 0xbc 188
{0x00,0x08,0x04,0x08,0x70,0x0c},   // Ѕ 0xbd 189
{0x00,0x60,0x50,0x48,0x50,0x60},   // ѕ 0xbe 190
{0x00,0x20,0x52,0x55,0x59,0x30},   // ї 0xbf 191
{0x00,0x38,0x54,0x54,0x54,0x00},   // А 0xc0 192
{0x00,0x00,0x00,0x7f,0x41,0x00},   // Б 0xc1 193
{0x00,0x40,0x22,0x14,0x18,0x60},   // В 0xc2 194
{0x00,0x7c,0x20,0x20,0x1c,0x20},   // Г 0xc3 195
{0x00,0x44,0x3c,0x04,0x7c,0x44},   // Д 0xc4 196
{0x00,0x40,0x3c,0x12,0x12,0x0c},   // Е 0xc5 197
{0x00,0x41,0x63,0x55,0x49,0x41},   // Ж 0xc6 198
{0x00,0x38,0x44,0x44,0x3c,0x04},   // З 0xc7 199
{0x00,0x08,0x04,0x3c,0x44,0x24},   // И 0xc8 200
{0x00,0x08,0x14,0x7f,0x14,0x08},   // Й 0xc9 201
{0x00,0x4e,0x71,0x01,0x71,0x4e},   // К 0xca 202
{0x00,0x45,0x29,0x11,0x29,0x45},   // Л 0xcb 203
{0x00,0x0d,0x51,0x51,0x51,0x3d},   // М 0xcc 204
{0x00,0x00,0x00,0x05,0x02,0x05},   // Н 0xcd 205
{0x00,0x40,0x00,0x40,0x00,0x40},   // О 0xce 206
{0x00,0x00,0x08,0x1c,0x3e,0x00},   // П 0xcf 207
{0x00,0x1c,0x1c,0x1c,0x00,0x00},   // Р 0xd0 208
{0x00,0x00,0x70,0x08,0x07,0x00},   // С 0xd1 209
{0x00,0x00,0x08,0x08,0x08,0x00},   // Т 0xd2 210
{0x00,0x00,0x1d,0x15,0x17,0x00},   // У 0xd3 211
{0x00,0x00,0x07,0x05,0x07,0x00},   // Ф 0xd4 212
{0x00,0x00,0x11,0x15,0x0a,0x00},   // Х 0xd5 213
{0x00,0x00,0x00,0x00,0x00,0x00},   // Ц 0xd6 214
{0x00,0x04,0x3c,0x41,0x20,0x00},   // Ч 0xd7 215
{0x00,0x7c,0x16,0x15,0x16,0x08},   // Ш 0xd8 216
{0x00,0x21,0x16,0x08,0x34,0x42},   // Щ 0xd9 217
{0x00,0x7f,0x09,0x1d,0x01,0x03},   // Ъ 0xda 218
{0x00,0x38,0x54,0x54,0x14,0x08},   // Ы 0xdb 219
{0x00,0x00,0x00,0x7c,0x40,0x40},   // Ь 0xdc 220
{0x00,0x7f,0x0e,0x1c,0x38,0x7f},   // Э 0xdd 221
{0x00,0x41,0x22,0x5d,0x22,0x1c},   // Ю 0xde 222
{0x00,0x1c,0x3e,0x1c,0x08,0x00},   // Я 0xdf 223
{0x00,0x7f,0x7f,0x7f,0x7f,0x7f},   // а 0xe0 224
{0x00,0x77,0x7b,0x01,0x7b,0x77},   // б 0xe1 225
{0x00,0x7f,0x43,0x75,0x43,0x7f},   // в 0xe2 226
{0x00,0x7f,0x6f,0x55,0x43,0x7f},   // г 0xe3 227
{0x00,0x40,0x40,0x40,0x40,0x40},   // д 0xe4 228
{0x00,0x44,0x42,0x5f,0x42,0x44},   // е 0xe5 229
{0x00,0x40,0x5e,0x45,0x5e,0x40},   // ж 0xe6 230
{0x00,0x40,0x48,0x55,0x5e,0x40},   // з 0xe7 231
{0x00,0x00,0x04,0x08,0x10,0x20},   // и 0xe8 232
{0x00,0x03,0x07,0x0e,0x1c,0x38},   // й 0xe9 233
{0x00,0x01,0x03,0x07,0x0f,0x1f},   // к 0xea 234
{0x00,0x7c,0x78,0x70,0x60,0x40},   // л 0xeb 235
{0x00,0x08,0x08,0x1c,0x22,0x1c},   // м 0xec 236
{0x00,0x00,0x1c,0x22,0x1c,0x00},   // н 0xed 237
{0x00,0x02,0x00,0x08,0x00,0x20},   // о 0xee 238
{0x00,0x04,0x3e,0x3f,0x3e,0x04},   // п 0xef 239
{0x00,0x10,0x3e,0x7e,0x3e,0x10},   // р 0xf0 240
{0x00,0x55,0x2a,0x55,0x2a,0x55},   // с 0xf1 241
{0x00,0x24,0x2a,0x7f,0x2a,0x12},   // т 0xf2 242
{0x00,0x04,0x1e,0x1f,0x1e,0x04},   // у 0xf3 243
{0x00,0x00,0x00,0x00,0x00,0x00},   // ф 0xf4 244
{0x00,0x00,0x00,0x00,0x00,0x00},   // х 0xf5 245
{0x00,0x00,0x00,0x00,0x00,0x00},   // ц 0xf6 246
{0x00,0x00,0x00,0x00,0x00,0x00},   // ч 0xf7 247
{0x00,0x00,0x00,0x00,0x00,0x00},   // ш 0xf8 248
{0x00,0x00,0x00,0x00,0x00,0x00},   // щ 0xf9 249
{0x00,0x00,0x00,0x00,0x00,0x00},   // ъ 0xfa 250
{0x00,0x00,0x00,0x00,0x00,0x00},   // ы 0xfb 251
{0x00,0x00,0x00,0x00,0x00,0x00},   // ь 0xfc 252
{0x00,0x00,0x00,0x00,0x00,0x00},   // э 0xfd 253
{0x00,0x00,0x00,0x00,0x00,0x00},   // ю 0xfe 254
{0x00,0x00,0x00,0x00,0x00,0x00}    // я 0xff 255
};

const uint8_t font8x12[][13] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //0
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //10
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //20
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //21
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //22
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //23
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //24
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //25
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //26
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //27
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //28
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //29
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //30
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //31
 
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //0x20 32
  {0x00,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x10,0x00,0x00,0x00,0x00}, 
  {0x00,0x6C,0x6C,0x24,0x24,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
  {0x48,0x48,0x24,0x7E,0x24,0x24,0x7E,0x24,0x12,0x12,0x00,0x00,0x00},
  {0x10,0x38,0x24,0x04,0x18,0x20,0x24,0x1C,0x10,0x10,0x00,0x00,0x00},
  {0x00,0x04,0x0A,0x04,0x30,0x0E,0x10,0x28,0x10,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x70,0x08,0x08,0x18,0x54,0x24,0x78,0x00,0x00,0x00,0x00},
  {0x00,0x08,0x08,0x08,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x20,0x20,0x10,0x10,0x10,0x10,0x10,0x10,0x20,0x20,0x00,0x00},
  {0x00,0x04,0x04,0x08,0x08,0x08,0x08,0x08,0x08,0x04,0x04,0x00,0x00},
  {0x00,0x10,0x7C,0x10,0x28,0x28,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x10,0x10,0x10,0xFE,0x10,0x10,0x10,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x08,0x0C,0x04,0x00,0x00},
  {0x00,0x00,0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00},
  {0x20,0x20,0x10,0x10,0x08,0x08,0x04,0x04,0x02,0x02,0x00,0x00,0x00},
  {0x00,0x3C,0x42,0x42,0x42,0x42,0x42,0x42,0x3C,0x00,0x00,0x00,0x00},
  {0x00,0x10,0x1C,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00,0x00,0x00},
  {0x00,0x1C,0x22,0x20,0x10,0x08,0x04,0x22,0x3E,0x00,0x00,0x00,0x00},
  {0x00,0x1C,0x22,0x20,0x18,0x20,0x20,0x22,0x1C,0x00,0x00,0x00,0x00},
  {0x00,0x30,0x28,0x24,0x24,0x7E,0x20,0x20,0x70,0x00,0x00,0x00,0x00},
  {0x00,0x7C,0x04,0x04,0x3C,0x40,0x40,0x42,0x3C,0x00,0x00,0x00,0x00},
  {0x00,0x70,0x08,0x04,0x3C,0x44,0x44,0x44,0x38,0x00,0x00,0x00,0x00},   //54
  {0x00,0x7E,0x42,0x40,0x20,0x20,0x10,0x10,0x10,0x00,0x00,0x00,0x00},
  {0x00,0x3C,0x42,0x42,0x3C,0x42,0x42,0x42,0x3C,0x00,0x00,0x00,0x00},
  {0x00,0x3C,0x42,0x42,0x42,0x7C,0x40,0x20,0x1E,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x18,0x0C,0x04,0x00,0x00,0x00},
  {0x00,0x00,0x40,0x30,0x08,0x06,0x08,0x30,0x40,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x02,0x0C,0x10,0x60,0x10,0x0C,0x02,0x00,0x00,0x00,0x00},
  {0x00,0x38,0x44,0x40,0x40,0x20,0x10,0x00,0x18,0x00,0x00,0x00,0x00},
  {0x1C,0x22,0x22,0x32,0x2A,0x2A,0x32,0x02,0x22,0x1C,0x00,0x00,0x00},
  {0x00,0x18,0x10,0x28,0x28,0x28,0x38,0x44,0xEE,0x00,0x00,0x00,0x00},
  {0x00,0x3E,0x44,0x44,0x3C,0x44,0x44,0x44,0x3E,0x00,0x00,0x00,0x00},
  {0x00,0x78,0x44,0x02,0x02,0x02,0x02,0x44,0x38,0x00,0x00,0x00,0x00},
  {0x00,0x1E,0x24,0x44,0x44,0x44,0x44,0x24,0x1E,0x00,0x00,0x00,0x00},
  {0x00,0x7E,0x44,0x14,0x1C,0x14,0x04,0x44,0x7E,0x00,0x00,0x00,0x00},
  {0x00,0x7E,0x44,0x14,0x1C,0x14,0x04,0x04,0x0E,0x00,0x00,0x00,0x00},
  {0x00,0x78,0x44,0x02,0x02,0xE2,0x42,0x44,0x38,0x00,0x00,0x00,0x00},
  {0x00,0xEE,0x44,0x44,0x7C,0x44,0x44,0x44,0xEE,0x00,0x00,0x00,0x00},
  {0x00,0x7C,0x10,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00,0x00,0x00},
  {0x00,0x78,0x20,0x20,0x20,0x22,0x22,0x22,0x1C,0x00,0x00,0x00,0x00},
  {0x00,0xEE,0x44,0x24,0x14,0x1C,0x24,0x44,0xCE,0x00,0x00,0x00,0x00},
  {0x00,0x0E,0x04,0x04,0x04,0x04,0x44,0x44,0x7E,0x00,0x00,0x00,0x00},
  {0x00,0xEE,0x6C,0x6C,0x54,0x54,0x44,0x44,0xEE,0x00,0x00,0x00,0x00},
  {0x00,0xE7,0x46,0x4A,0x4A,0x52,0x52,0x62,0x67,0x00,0x00,0x00,0x00},
  {0x00,0x38,0x44,0x82,0x82,0x82,0x82,0x44,0x38,0x00,0x00,0x00,0x00},
  {0x00,0x3E,0x44,0x44,0x44,0x3C,0x04,0x04,0x0E,0x00,0x00,0x00,0x00},
  {0x00,0x38,0x44,0x82,0x82,0x82,0x82,0x44,0x38,0xF8,0x00,0x00,0x00},
  {0x00,0x3E,0x44,0x44,0x44,0x3C,0x24,0x44,0x8E,0x00,0x00,0x00,0x00},
  {0x00,0x5C,0x62,0x02,0x3C,0x40,0x40,0x46,0x3A,0x00,0x00,0x00,0x00},
  {0x00,0xFE,0x92,0x10,0x10,0x10,0x10,0x10,0x38,0x00,0x00,0x00,0x00}, //0x54
  {0x00,0xEE,0x44,0x44,0x44,0x44,0x44,0x44,0x38,0x00,0x00,0x00,0x00},
  {0x00,0xE7,0x42,0x42,0x24,0x24,0x24,0x18,0x18,0x00,0x00,0x00,0x00},
  {0x00,0xEE,0x44,0x44,0x54,0x54,0x54,0x54,0x28,0x00,0x00,0x00,0x00},
  {0x00,0xEE,0x44,0x28,0x10,0x10,0x28,0x44,0xEE,0x00,0x00,0x00,0x00},
  {0x00,0xEE,0x44,0x28,0x28,0x10,0x10,0x10,0x38,0x00,0x00,0x00,0x00},
  {0x00,0x7C,0x44,0x20,0x10,0x10,0x08,0x44,0x7C,0x00,0x00,0x00,0x00},
  {0x00,0x38,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x38,0x00,0x00},
  {0x02,0x02,0x04,0x04,0x08,0x08,0x08,0x10,0x10,0x10,0x00,0x00,0x00},
  {0x00,0x0E,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x0E,0x00,0x00},
  {0x08,0x08,0x14,0x22,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF},
  {0x00,0x08,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x3C,0x42,0x7C,0x42,0x62,0xDC,0x00,0x00,0x00,0x00},
  {0x00,0x03,0x02,0x3A,0x46,0x42,0x42,0x46,0x3B,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x5C,0x62,0x02,0x02,0x42,0x3C,0x00,0x00,0x00,0x00},
  {0x00,0x60,0x40,0x5C,0x62,0x42,0x42,0x62,0xDC,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x3C,0x42,0x7E,0x02,0x02,0x7C,0x00,0x00,0x00,0x00},
  {0x00,0x70,0x08,0x7E,0x08,0x08,0x08,0x08,0x7E,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0xDC,0x62,0x42,0x42,0x62,0x5C,0x40,0x3C,0x00,0x00},
  {0x00,0x06,0x04,0x34,0x4C,0x44,0x44,0x44,0xEE,0x00,0x00,0x00,0x00},
  {0x00,0x10,0x00,0x1C,0x10,0x10,0x10,0x10,0x7C,0x00,0x00,0x00,0x00},
  {0x00,0x10,0x00,0x3C,0x20,0x20,0x20,0x20,0x20,0x20,0x1E,0x00,0x00},
  {0x00,0x06,0x04,0xF4,0x24,0x1C,0x14,0x24,0xE6,0x00,0x00,0x00,0x00},
  {0x00,0x18,0x10,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x4B,0xB6,0x92,0x92,0x92,0xB7,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x36,0x4C,0x44,0x44,0x44,0xEE,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x3C,0x42,0x42,0x42,0x42,0x3C,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x36,0x4C,0x44,0x44,0x44,0x3C,0x04,0x0E,0x00,0x00},
  {0x00,0x00,0x00,0xDC,0x62,0x42,0x42,0x62,0x5C,0x40,0xE0,0x00,0x00},
  {0x00,0x00,0x00,0x76,0x0C,0x04,0x04,0x04,0x3E,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x7C,0x42,0x3C,0x40,0x42,0x3E,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x04,0x3E,0x04,0x04,0x04,0x44,0x38,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x66,0x44,0x44,0x44,0x64,0xD8,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0xE7,0x42,0x24,0x24,0x18,0x18,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0xEE,0x44,0x54,0x54,0x54,0x28,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x66,0x24,0x18,0x18,0x24,0x66,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0xEE,0x44,0x44,0x28,0x28,0x10,0x10,0x1C,0x00,0x00},
  {0x00,0x00,0x00,0x7C,0x24,0x10,0x08,0x44,0x7C,0x00,0x00,0x00,0x00},
  {0x00,0x10,0x08,0x08,0x08,0x04,0x08,0x08,0x08,0x10,0x00,0x00,0x00},
  {0x00,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,0x00},
  {0x00,0x04,0x08,0x08,0x08,0x10,0x08,0x08,0x08,0x04,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x00,0x4C,0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
};

static void ili9163_reset(void);
static void ili9163_cmd(uint8_t address);
static void ili9163_param(uint8_t parameter);
static void ili9163_data(uint8_t dataByte1, uint8_t dataByte2);

// Low-level LCD driving functions --------------------------------------------------------------------------
static struct ili9163_operations *ili9163_ops;

inline uint16_t decodeRgbValue(uint8_t r, uint8_t g, uint8_t b)
{
	return (b << 11) | (g << 6) | (r);
}	

// Reset the LCD hardware
static void ili9163_reset(void)
{     
	// Reset pin is active low (0 = reset, 1 = ready)
	ili9163_ops->ili9163_write_reset(0);
    ili9163_ops->ili9163_delay_ms(50);
    ili9163_ops->ili9163_write_reset(1);
    ili9163_ops->ili9163_delay_ms(120);
}

static void ili9163_cmd(uint8_t address)
{   
	ili9163_ops->ili9163_write_cs(0);
	ili9163_ops->ili9163_write_datacom(0);
	ili9163_ops->ili9163_spi_send(&address);
	ili9163_ops->ili9163_write_cs(1);
}

static void ili9163_param(uint8_t parameter)
{  
	ili9163_ops->ili9163_write_cs(0);
	ili9163_ops->ili9163_write_datacom(1);
	ili9163_ops->ili9163_spi_send(&parameter);
	ili9163_ops->ili9163_write_cs(1);
}
 
static void ili9163_data(uint8_t dataByte1, uint8_t dataByte2)
{  
	ili9163_ops->ili9163_write_cs(0);
	ili9163_ops->ili9163_write_datacom(1);
	ili9163_ops->ili9163_spi_send(&dataByte1);
	ili9163_ops->ili9163_spi_send(&dataByte2);
	ili9163_ops->ili9163_write_cs(1);
}

uint8_t lcdTextX(uint8_t x) { return x*6; }
uint8_t lcdTextY(uint8_t y) { return y*8; }

// Initialise the display with the require screen orientation
void ili9163_init(struct ili9163_operations *init_ili9163, uint8_t orientation)
{   
	ili9163_ops = init_ili9163;

	// Hardware reset the LCD
	ili9163_reset();
	
    ili9163_cmd(EXIT_SLEEP_MODE);
    ili9163_ops->ili9163_delay_ms(5); // Wait for the screen to wake up
    
    ili9163_cmd(SET_PIXEL_FORMAT);
    ili9163_param(0x05); // 16 bits per pixel
   
    ili9163_cmd(SET_GAMMA_CURVE);
    ili9163_param(0x04); // Select gamma curve 3
	
    ili9163_cmd(GAM_R_SEL);
    ili9163_param(0x01); // Gamma adjustment enabled
    
    ili9163_cmd(POSITIVE_GAMMA_CORRECT);
    ili9163_param(0x3f); // 1st Parameter
    ili9163_param(0x25); // 2nd Parameter
    ili9163_param(0x1c); // 3rd Parameter
    ili9163_param(0x1e); // 4th Parameter
    ili9163_param(0x20); // 5th Parameter
    ili9163_param(0x12); // 6th Parameter
    ili9163_param(0x2a); // 7th Parameter
    ili9163_param(0x90); // 8th Parameter
    ili9163_param(0x24); // 9th Parameter
    ili9163_param(0x11); // 10th Parameter
    ili9163_param(0x00); // 11th Parameter
    ili9163_param(0x00); // 12th Parameter
    ili9163_param(0x00); // 13th Parameter
    ili9163_param(0x00); // 14th Parameter
    ili9163_param(0x00); // 15th Parameter
     
    ili9163_cmd(NEGATIVE_GAMMA_CORRECT);
    ili9163_param(0x20); // 1st Parameter
    ili9163_param(0x20); // 2nd Parameter
    ili9163_param(0x20); // 3rd Parameter
    ili9163_param(0x20); // 4th Parameter
    ili9163_param(0x05); // 5th Parameter
    ili9163_param(0x00); // 6th Parameter
    ili9163_param(0x15); // 7th Parameter
    ili9163_param(0xa7); // 8th Parameter
    ili9163_param(0x3d); // 9th Parameter
    ili9163_param(0x18); // 10th Parameter
    ili9163_param(0x25); // 11th Parameter
    ili9163_param(0x2a); // 12th Parameter
    ili9163_param(0x2b); // 13th Parameter
    ili9163_param(0x2b); // 14th Parameter
    ili9163_param(0x3a); // 15th Parameter
    
    ili9163_cmd(FRAME_RATE_CONTROL1);
    ili9163_param(0x08); // DIVA = 8
    ili9163_param(0x08); // VPA = 8
    
    ili9163_cmd(DISPLAY_INVERSION);
    ili9163_param(0x07); // NLA = 1, NLB = 1, NLC = 1 (all on Frame Inversion)
   
    ili9163_cmd(POWER_CONTROL1);
    ili9163_param(0x0a); // VRH = 10:  GVDD = 4.30
    ili9163_param(0x02); // VC = 2: VCI1 = 2.65
      
    ili9163_cmd(POWER_CONTROL2);
    ili9163_param(0x02); // BT = 2: AVDD = 2xVCI1, VCL = -1xVCI1, VGH = 5xVCI1, VGL = -2xVCI1

    ili9163_cmd(VCOM_CONTROL1);
    ili9163_param(0x50); // VMH = 80: VCOMH voltage = 4.5
    ili9163_param(0x5b); // VML = 91: VCOML voltage = -0.225
	
    ili9163_cmd(VCOM_OFFSET_CONTROL);
    ili9163_param(0x40); // nVM = 0, VMF = 64: VCOMH output = VMH, VCOML output = VML	
    
    ili9163_cmd(SET_COLUMN_ADDRESS);
    ili9163_param(0x00); // XSH
    ili9163_param(0x00); // XSL
    ili9163_param(0x00); // XEH
    ili9163_param(0x7f); // XEL (128 pixels x)

    ili9163_cmd(SET_PAGE_ADDRESS);
    ili9163_param(0x00);
    ili9163_param(0x00);
    ili9163_param(0x00);
    ili9163_param(0x9f); // 128 pixels y


	// Select display orientation
    ili9163_cmd(SET_ADDRESS_MODE);
	ili9163_param(orientation);

	// Set the display to on
    ili9163_cmd(SET_DISPLAY_ON);
    ili9163_cmd(WRITE_MEMORY_START);
}

// LCD graphics functions -----------------------------------------------------------------------------------

void ili9163_clear(uint16_t colour)
{
	uint16_t pixel;
  
	// Set the column address to 0-127
	ili9163_cmd(SET_COLUMN_ADDRESS);
	ili9163_param(0x00);
	ili9163_param(0x00);
	ili9163_param(0x00);
	ili9163_param(0x7f);

	// Set the page address to 0-127
	ili9163_cmd(SET_PAGE_ADDRESS);
	ili9163_param(0x00);
	ili9163_param(0x00);
	ili9163_param(0x00);
	ili9163_param(0x7f);
  
	// Plot the pixels
	ili9163_cmd(WRITE_MEMORY_START);
	for(pixel = 0; pixel < 16385; pixel++) ili9163_data(colour >> 8, colour);
}

void ili9163_plot(uint8_t x, uint8_t y, uint16_t colour)
{
	// Horizontal Address Start Position
	ili9163_cmd(SET_COLUMN_ADDRESS);
	ili9163_param(0x00);
	ili9163_param(x);
	ili9163_param(0x00);
	ili9163_param(0x7f);
  
	// Vertical Address end Position
	ili9163_cmd(SET_PAGE_ADDRESS);
	ili9163_param(0x00);
	ili9163_param(y);
	ili9163_param(0x00);
	ili9163_param(0x7f);

	// Plot the point
	ili9163_cmd(WRITE_MEMORY_START);
	ili9163_data(colour >> 8, colour);
}

// Draw a line from x0, y0 to x1, y1
// Note:	This is a version of Bresenham's line drawing algorithm
//			It only draws lines from left to right!
void ili9163_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t colour)
{
	int16_t dy = y1 - y0;
	int16_t dx = x1 - x0;
	int16_t stepx, stepy;

	if (dy < 0)
	{
		dy = -dy; stepy = -1; 
	}
	else stepy = 1; 

 	if (dx < 0)
	{
		dx = -dx; stepx = -1; 
	}
	else stepx = 1; 

	dy <<= 1; 							// dy is now 2*dy
	dx <<= 1; 							// dx is now 2*dx
 
	ili9163_plot(x0, y0, colour);

	if (dx > dy) {
		int fraction = dy - (dx >> 1);	// same as 2*dy - dx
		while (x0 != x1)
		{
			if (fraction >= 0)
			{
				y0 += stepy;
				fraction -= dx; 		// same as fraction -= 2*dx
			}

   			x0 += stepx;
   			fraction += dy; 				// same as fraction -= 2*dy
   			ili9163_plot(x0, y0, colour);
		}
	}
	else
	{
		int fraction = dx - (dy >> 1);
		while (y0 != y1)
		{
			if (fraction >= 0)
			{
				x0 += stepx;
				fraction -= dy;
			}

			y0 += stepy;
			fraction += dx;
			ili9163_plot(x0, y0, colour);
		}
	}
}

// Draw a rectangle between x0, y0 and x1, y1
void ili9163_rectangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t colour)
{
	ili9163_line(x0, y0, x0, y1, colour);
	ili9163_line(x0, y1, x1, y1, colour);
	ili9163_line(x1, y0, x1, y1, colour);
	ili9163_line(x0, y0, x1, y0, colour);
}

// Draw a filled rectangle
// Note:	y1 must be greater than y0  and x1 must be greater than x0
//			for this to work
void ili9163_FilledRectangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t colour)
{
	uint16_t pixels;
			
	// To speed up plotting we define a x window with the width of the 
	// rectangle and then just output the required number of bytes to
	// fill down to the end point
	
	ili9163_cmd(SET_COLUMN_ADDRESS); // Horizontal Address Start Position
	ili9163_param(0x00);
	ili9163_param(x0);
	ili9163_param(0x00);
	ili9163_param(x1);
  
	ili9163_cmd(SET_PAGE_ADDRESS); // Vertical Address end Position
	ili9163_param(0x00);
	ili9163_param(y0);
	ili9163_param(0x00);
	ili9163_param(0x7f);
		
	ili9163_cmd(WRITE_MEMORY_START);
	
	for (pixels = 0; pixels < ((x1 - x0) * (y1 - y0)); pixels++)
		ili9163_data(colour >> 8, colour);;
}

// Draw a circle
// Note:	This is another version of Bresenham's line drawing algorithm.
//			There's plenty of documentation on the web if you are curious
//			how this works.
void ili9163_circle(int16_t xCentre, int16_t yCentre, int16_t radius, uint16_t colour)
{
	int16_t x = 0, y = radius;
	int16_t d = 3 - (2 * radius);
 
    while(x <= y)
	{
		ili9163_plot(xCentre + x, yCentre + y, colour);
		ili9163_plot(xCentre + y, yCentre + x, colour);
		ili9163_plot(xCentre - x, yCentre + y, colour);
		ili9163_plot(xCentre + y, yCentre - x, colour);
		ili9163_plot(xCentre - x, yCentre - y, colour);
		ili9163_plot(xCentre - y, yCentre - x, colour);
		ili9163_plot(xCentre + x, yCentre - y, colour);
		ili9163_plot(xCentre - y, yCentre + x, colour);

		if (d < 0) d += (4 * x) + 6;
		else
		{
			d += (4 * (x - y)) + 10;
			y -= 1;
		}

		x++;
	}
}

// LCD text manipulation functions --------------------------------------------------------------------------

// Plot a character at the specified x, y co-ordinates (top left hand corner of character)
void ili9163_putchar(unsigned char character, uint8_t x, uint8_t y, uint16_t fgColour, uint16_t bgColour)
{
	uint8_t row, column;
	
	// To speed up plotting we define a x window of 6 pixels and then
	// write out one row at a time.  This means the LCD will correctly
	// update the memory pointer saving us a good few bytes
	ili9163_cmd(SET_COLUMN_ADDRESS); // Horizontal Address Start Position
	ili9163_param(0x00);
	ili9163_param(x);
	ili9163_param(0x00);
    
#ifdef FONT8x5
	ili9163_param(x+5);
#else
	ili9163_param(x+7);
#endif
	ili9163_cmd(SET_PAGE_ADDRESS); // Vertical Address end Position
	ili9163_param(0x00);
	ili9163_param(y);
	ili9163_param(0x00);
	ili9163_param(0x7f);
		
	ili9163_cmd(WRITE_MEMORY_START);
	
	// Plot the font data
#ifdef FONT8x5
	for (row = 0; row < 8; row++)
	{
		for (column = 0; column < 6; column++)
		{
			if (font8x5[character][column] & (1 << row))
				ili9163_data(fgColour >> 8, fgColour);
			else ili9163_data(bgColour >> 8, bgColour);
		}
	}
#else
	for (row = 0; row < 13; row++)
	{
		uint8_t temp = font8x12[character][row];
		for (column = 0; column < 8; column++)
		{
			if (temp & 0x01)
				ili9163_data(fgColour >> 8, fgColour);
			else
				ili9163_data(bgColour >> 8, bgColour);
			temp >>= 1;
		}
	}
#endif
}

// Plot a string of characters to the LCD
void ili9163_puts(const char *string, uint8_t x, uint8_t y, uint16_t fgColour, uint16_t bgColour)
{
	uint8_t origin = x;

	for (uint8_t characterNumber = 0; characterNumber < strlen(string); characterNumber++)
	{
		// Check if we are out of bounds and move to 
		// the next line if we are
		if (x > 121)
		{
			x = origin;
			y += 8;
		}

		// If we move past the bottom of the screen just exit
		if (y > 120) break;

		// Plot the current character
		ili9163_putchar(string[characterNumber], x, y, fgColour, bgColour);
#ifdef FONT8x5
		x += 6;
#else        
        x += 9;
        #endif
	}
}


