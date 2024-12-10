#include "include_04.h"

#define U_0   0x0c00     //  0---4095�ĵ�Ƕȿ��Ʒ��Ķβ�ѯ90
#define U0_90  0x0000
#define U90_180  0x0400
#define U180_270 0x0800
#define U270_360 0x0c00

Ang_SinCos Ang_SinCosPark = Ang_SinCos_DEFAULTS;

const Uint16 fSin[1025] =       //IQ10��ʽsin��
        { 0X0, 0X2, 0X3, 0X5, 0X6, 0X8, 0X9, 0XB, 0XD, 0XE, 0X10, 0X11, 0X13,
          0X14, 0X16, 0X18, 0X19, 0X1B, 0X1C, 0X1E, 0X1F, 0X21, 0X23, 0X24,
          0X26, 0X27, 0X29, 0X2A, 0X2C, 0X2E, 0X2F, 0X31, 0X32, 0X34, 0X35,
          0X37, 0X39, 0X3A, 0X3C, 0X3D, 0X3F, 0X40, 0X42, 0X43, 0X45, 0X47,
          0X48, 0X4A, 0X4B, 0X4D, 0X4E, 0X50, 0X52, 0X53, 0X55, 0X56, 0X58,
          0X59, 0X5B, 0X5D, 0X5E, 0X60, 0X61, 0X63, 0X64, 0X66, 0X67, 0X69,
          0X6B, 0X6C, 0X6E, 0X6F, 0X71, 0X72, 0X74, 0X76, 0X77, 0X79, 0X7A,
          0X7C, 0X7D, 0X7F, 0X80, 0X82, 0X84, 0X85, 0X87, 0X88, 0X8A, 0X8B,
          0X8D, 0X8E, 0X90, 0X92, 0X93, 0X95, 0X96, 0X98, 0X99, 0X9B, 0X9C,
          0X9E, 0XA0, 0XA1, 0XA3, 0XA4, 0XA6, 0XA7, 0XA9, 0XAA, 0XAC, 0XAE,
          0XAF, 0XB1, 0XB2, 0XB4, 0XB5, 0XB7, 0XB8, 0XBA, 0XBB, 0XBD, 0XBF,
          0XC0, 0XC2, 0XC3, 0XC5, 0XC6, 0XC8, 0XC9, 0XCB, 0XCC, 0XCE, 0XCF,
          0XD1, 0XD3, 0XD4, 0XD6, 0XD7, 0XD9, 0XDA, 0XDC, 0XDD, 0XDF, 0XE0,
          0XE2, 0XE3, 0XE5, 0XE6, 0XE8, 0XEA, 0XEB, 0XED, 0XEE, 0XF0, 0XF1,
          0XF3, 0XF4, 0XF6, 0XF7, 0XF9, 0XFA, 0XFC, 0XFD, 0XFF, 0X100, 0X102,
          0X103, 0X105, 0X107, 0X108, 0X10A, 0X10B, 0X10D, 0X10E, 0X110, 0X111,
          0X113, 0X114, 0X116, 0X117, 0X119, 0X11A, 0X11C, 0X11D, 0X11F, 0X120,
          0X122, 0X123, 0X125, 0X126, 0X128, 0X129, 0X12B, 0X12C, 0X12E, 0X12F,
          0X131, 0X132, 0X134, 0X135, 0X137, 0X138, 0X13A, 0X13B, 0X13D, 0X13E,
          0X140, 0X141, 0X143, 0X144, 0X146, 0X147, 0X149, 0X14A, 0X14C, 0X14D,
          0X14F, 0X150, 0X152, 0X153, 0X155, 0X156, 0X157, 0X159, 0X15A, 0X15C,
          0X15D, 0X15F, 0X160, 0X162, 0X163, 0X165, 0X166, 0X168, 0X169, 0X16B,
          0X16C, 0X16E, 0X16F, 0X171, 0X172, 0X173, 0X175, 0X176, 0X178, 0X179,
          0X17B, 0X17C, 0X17E, 0X17F, 0X181, 0X182, 0X184, 0X185, 0X186, 0X188,
          0X189, 0X18B, 0X18C, 0X18E, 0X18F, 0X191, 0X192, 0X193, 0X195, 0X196,
          0X198, 0X199, 0X19B, 0X19C, 0X19E, 0X19F, 0X1A0, 0X1A2, 0X1A3, 0X1A5,
          0X1A6, 0X1A8, 0X1A9, 0X1AA, 0X1AC, 0X1AD, 0X1AF, 0X1B0, 0X1B2, 0X1B3,
          0X1B4, 0X1B6, 0X1B7, 0X1B9, 0X1BA, 0X1BB, 0X1BD, 0X1BE, 0X1C0, 0X1C1,
          0X1C3, 0X1C4, 0X1C5, 0X1C7, 0X1C8, 0X1CA, 0X1CB, 0X1CC, 0X1CE, 0X1CF,
          0X1D1, 0X1D2, 0X1D3, 0X1D5, 0X1D6, 0X1D8, 0X1D9, 0X1DA, 0X1DC, 0X1DD,
          0X1DF, 0X1E0, 0X1E1, 0X1E3, 0X1E4, 0X1E5, 0X1E7, 0X1E8, 0X1EA, 0X1EB,
          0X1EC, 0X1EE, 0X1EF, 0X1F1, 0X1F2, 0X1F3, 0X1F5, 0X1F6, 0X1F7, 0X1F9,
          0X1FA, 0X1FB, 0X1FD, 0X1FE, 0X200, 0X201, 0X202, 0X204, 0X205, 0X206,
          0X208, 0X209, 0X20A, 0X20C, 0X20D, 0X20E, 0X210, 0X211, 0X212, 0X214,
          0X215, 0X217, 0X218, 0X219, 0X21B, 0X21C, 0X21D, 0X21F, 0X220, 0X221,
          0X223, 0X224, 0X225, 0X226, 0X228, 0X229, 0X22A, 0X22C, 0X22D, 0X22E,
          0X230, 0X231, 0X232, 0X234, 0X235, 0X236, 0X238, 0X239, 0X23A, 0X23C,
          0X23D, 0X23E, 0X23F, 0X241, 0X242, 0X243, 0X245, 0X246, 0X247, 0X248,
          0X24A, 0X24B, 0X24C, 0X24E, 0X24F, 0X250, 0X251, 0X253, 0X254, 0X255,
          0X257, 0X258, 0X259, 0X25A, 0X25C, 0X25D, 0X25E, 0X25F, 0X261, 0X262,
          0X263, 0X265, 0X266, 0X267, 0X268, 0X26A, 0X26B, 0X26C, 0X26D, 0X26F,
          0X270, 0X271, 0X272, 0X274, 0X275, 0X276, 0X277, 0X278, 0X27A, 0X27B,
          0X27C, 0X27D, 0X27F, 0X280, 0X281, 0X282, 0X284, 0X285, 0X286, 0X287,
          0X288, 0X28A, 0X28B, 0X28C, 0X28D, 0X28E, 0X290, 0X291, 0X292, 0X293,
          0X294, 0X296, 0X297, 0X298, 0X299, 0X29A, 0X29C, 0X29D, 0X29E, 0X29F,
          0X2A0, 0X2A2, 0X2A3, 0X2A4, 0X2A5, 0X2A6, 0X2A7, 0X2A9, 0X2AA, 0X2AB,
          0X2AC, 0X2AD, 0X2AF, 0X2B0, 0X2B1, 0X2B2, 0X2B3, 0X2B4, 0X2B5, 0X2B7,
          0X2B8, 0X2B9, 0X2BA, 0X2BB, 0X2BC, 0X2BE, 0X2BF, 0X2C0, 0X2C1, 0X2C2,
          0X2C3, 0X2C4, 0X2C5, 0X2C7, 0X2C8, 0X2C9, 0X2CA, 0X2CB, 0X2CC, 0X2CD,
          0X2CF, 0X2D0, 0X2D1, 0X2D2, 0X2D3, 0X2D4, 0X2D5, 0X2D6, 0X2D7, 0X2D9,
          0X2DA, 0X2DB, 0X2DC, 0X2DD, 0X2DE, 0X2DF, 0X2E0, 0X2E1, 0X2E2, 0X2E3,
          0X2E5, 0X2E6, 0X2E7, 0X2E8, 0X2E9, 0X2EA, 0X2EB, 0X2EC, 0X2ED, 0X2EE,
          0X2EF, 0X2F0, 0X2F1, 0X2F3, 0X2F4, 0X2F5, 0X2F6, 0X2F7, 0X2F8, 0X2F9,
          0X2FA, 0X2FB, 0X2FC, 0X2FD, 0X2FE, 0X2FF, 0X300, 0X301, 0X302, 0X303,
          0X304, 0X305, 0X306, 0X307, 0X308, 0X309, 0X30A, 0X30B, 0X30C, 0X30E,
          0X30F, 0X310, 0X311, 0X312, 0X313, 0X314, 0X315, 0X316, 0X317, 0X318,
          0X319, 0X31A, 0X31B, 0X31C, 0X31D, 0X31E, 0X31E, 0X31F, 0X320, 0X321,
          0X322, 0X323, 0X324, 0X325, 0X326, 0X327, 0X328, 0X329, 0X32A, 0X32B,
          0X32C, 0X32D, 0X32E, 0X32F, 0X330, 0X331, 0X332, 0X333, 0X334, 0X335,
          0X336, 0X336, 0X337, 0X338, 0X339, 0X33A, 0X33B, 0X33C, 0X33D, 0X33E,
          0X33F, 0X340, 0X341, 0X342, 0X342, 0X343, 0X344, 0X345, 0X346, 0X347,
          0X348, 0X349, 0X34A, 0X34B, 0X34B, 0X34C, 0X34D, 0X34E, 0X34F, 0X350,
          0X351, 0X352, 0X353, 0X353, 0X354, 0X355, 0X356, 0X357, 0X358, 0X359,
          0X359, 0X35A, 0X35B, 0X35C, 0X35D, 0X35E, 0X35F, 0X35F, 0X360, 0X361,
          0X362, 0X363, 0X364, 0X364, 0X365, 0X366, 0X367, 0X368, 0X369, 0X369,
          0X36A, 0X36B, 0X36C, 0X36D, 0X36E, 0X36E, 0X36F, 0X370, 0X371, 0X372,
          0X372, 0X373, 0X374, 0X375, 0X375, 0X376, 0X377, 0X378, 0X379, 0X379,
          0X37A, 0X37B, 0X37C, 0X37D, 0X37D, 0X37E, 0X37F, 0X380, 0X380, 0X381,
          0X382, 0X383, 0X383, 0X384, 0X385, 0X386, 0X386, 0X387, 0X388, 0X389,
          0X389, 0X38A, 0X38B, 0X38B, 0X38C, 0X38D, 0X38E, 0X38E, 0X38F, 0X390,
          0X391, 0X391, 0X392, 0X393, 0X393, 0X394, 0X395, 0X395, 0X396, 0X397,
          0X398, 0X398, 0X399, 0X39A, 0X39A, 0X39B, 0X39C, 0X39C, 0X39D, 0X39E,
          0X39E, 0X39F, 0X3A0, 0X3A0, 0X3A1, 0X3A2, 0X3A2, 0X3A3, 0X3A4, 0X3A4,
          0X3A5, 0X3A6, 0X3A6, 0X3A7, 0X3A8, 0X3A8, 0X3A9, 0X3A9, 0X3AA, 0X3AB,
          0X3AB, 0X3AC, 0X3AD, 0X3AD, 0X3AE, 0X3AE, 0X3AF, 0X3B0, 0X3B0, 0X3B1,
          0X3B1, 0X3B2, 0X3B3, 0X3B3, 0X3B4, 0X3B4, 0X3B5, 0X3B6, 0X3B6, 0X3B7,
          0X3B7, 0X3B8, 0X3B9, 0X3B9, 0X3BA, 0X3BA, 0X3BB, 0X3BB, 0X3BC, 0X3BD,
          0X3BD, 0X3BE, 0X3BE, 0X3BF, 0X3BF, 0X3C0, 0X3C0, 0X3C1, 0X3C1, 0X3C2,
          0X3C3, 0X3C3, 0X3C4, 0X3C4, 0X3C5, 0X3C5, 0X3C6, 0X3C6, 0X3C7, 0X3C7,
          0X3C8, 0X3C8, 0X3C9, 0X3C9, 0X3CA, 0X3CA, 0X3CB, 0X3CB, 0X3CC, 0X3CC,
          0X3CD, 0X3CD, 0X3CE, 0X3CE, 0X3CF, 0X3CF, 0X3D0, 0X3D0, 0X3D1, 0X3D1,
          0X3D2, 0X3D2, 0X3D3, 0X3D3, 0X3D3, 0X3D4, 0X3D4, 0X3D5, 0X3D5, 0X3D6,
          0X3D6, 0X3D7, 0X3D7, 0X3D7, 0X3D8, 0X3D8, 0X3D9, 0X3D9, 0X3DA, 0X3DA,
          0X3DA, 0X3DB, 0X3DB, 0X3DC, 0X3DC, 0X3DD, 0X3DD, 0X3DD, 0X3DE, 0X3DE,
          0X3DF, 0X3DF, 0X3DF, 0X3E0, 0X3E0, 0X3E1, 0X3E1, 0X3E1, 0X3E2, 0X3E2,
          0X3E2, 0X3E3, 0X3E3, 0X3E4, 0X3E4, 0X3E4, 0X3E5, 0X3E5, 0X3E5, 0X3E6,
          0X3E6, 0X3E6, 0X3E7, 0X3E7, 0X3E7, 0X3E8, 0X3E8, 0X3E8, 0X3E9, 0X3E9,
          0X3E9, 0X3EA, 0X3EA, 0X3EA, 0X3EB, 0X3EB, 0X3EB, 0X3EC, 0X3EC, 0X3EC,
          0X3ED, 0X3ED, 0X3ED, 0X3EE, 0X3EE, 0X3EE, 0X3EE, 0X3EF, 0X3EF, 0X3EF,
          0X3F0, 0X3F0, 0X3F0, 0X3F0, 0X3F1, 0X3F1, 0X3F1, 0X3F1, 0X3F2, 0X3F2,
          0X3F2, 0X3F2, 0X3F3, 0X3F3, 0X3F3, 0X3F3, 0X3F4, 0X3F4, 0X3F4, 0X3F4,
          0X3F5, 0X3F5, 0X3F5, 0X3F5, 0X3F6, 0X3F6, 0X3F6, 0X3F6, 0X3F6, 0X3F7,
          0X3F7, 0X3F7, 0X3F7, 0X3F8, 0X3F8, 0X3F8, 0X3F8, 0X3F8, 0X3F8, 0X3F9,
          0X3F9, 0X3F9, 0X3F9, 0X3F9, 0X3FA, 0X3FA, 0X3FA, 0X3FA, 0X3FA, 0X3FA,
          0X3FB, 0X3FB, 0X3FB, 0X3FB, 0X3FB, 0X3FB, 0X3FC, 0X3FC, 0X3FC, 0X3FC,
          0X3FC, 0X3FC, 0X3FC, 0X3FC, 0X3FD, 0X3FD, 0X3FD, 0X3FD, 0X3FD, 0X3FD,
          0X3FD, 0X3FD, 0X3FE, 0X3FE, 0X3FE, 0X3FE, 0X3FE, 0X3FE, 0X3FE, 0X3FE,
          0X3FE, 0X3FE, 0X3FF, 0X3FF, 0X3FF, 0X3FF, 0X3FF, 0X3FF, 0X3FF, 0X3FF,
          0X3FF, 0X3FF, 0X3FF, 0X3FF, 0X3FF, 0X3FF, 0X3FF, 0X400, 0X400, 0X400,
          0X400, 0X400, 0X400, 0X400, 0X400, 0X400, 0X400, 0X400, 0X400, 0X400,
          0X400, 0X400, 0X400, 0X400, 0X400, 0X400, 0X400, };

const Uint32 ftan[1025] =           //IQ10��ʽtan����
        { 0X0, 0X1, 0X3, 0X4, 0X6, 0X7, 0X9, 0XA, 0XC, 0XE, 0XF, 0X11, 0X12,
          0X14, 0X15, 0X17, 0X19, 0X1A, 0X1C, 0X1D, 0X1F, 0X20, 0X22, 0X24,
          0X25, 0X27, 0X28, 0X2A, 0X2C, 0X2D, 0X2F, 0X30, 0X32, 0X33, 0X35,
          0X37, 0X38, 0X3A, 0X3B, 0X3D, 0X3E, 0X40, 0X42, 0X43, 0X45, 0X46,
          0X48, 0X49, 0X4B, 0X4D, 0X4E, 0X50, 0X51, 0X53, 0X55, 0X56, 0X58,
          0X59, 0X5B, 0X5C, 0X5E, 0X60, 0X61, 0X63, 0X64, 0X66, 0X68, 0X69,
          0X6B, 0X6C, 0X6E, 0X6F, 0X71, 0X73, 0X74, 0X76, 0X77, 0X79, 0X7B,
          0X7C, 0X7E, 0X7F, 0X81, 0X83, 0X84, 0X86, 0X87, 0X89, 0X8B, 0X8C,
          0X8E, 0X8F, 0X91, 0X93, 0X94, 0X96, 0X97, 0X99, 0X9B, 0X9C, 0X9E,
          0X9F, 0XA1, 0XA3, 0XA4, 0XA6, 0XA7, 0XA9, 0XAB, 0XAC, 0XAE, 0XB0,
          0XB1, 0XB3, 0XB4, 0XB6, 0XB8, 0XB9, 0XBB, 0XBD, 0XBE, 0XC0, 0XC1,
          0XC3, 0XC5, 0XC6, 0XC8, 0XCA, 0XCB, 0XCD, 0XCE, 0XD0, 0XD2, 0XD3,
          0XD5, 0XD7, 0XD8, 0XDA, 0XDC, 0XDD, 0XDF, 0XE1, 0XE2, 0XE4, 0XE5,
          0XE7, 0XE9, 0XEA, 0XEC, 0XEE, 0XEF, 0XF1, 0XF3, 0XF4, 0XF6, 0XF8,
          0XF9, 0XFB, 0XFD, 0XFE, 0X100, 0X102, 0X103, 0X105, 0X107, 0X108,
          0X10A, 0X10C, 0X10D, 0X10F, 0X111, 0X112, 0X114, 0X116, 0X117, 0X119,
          0X11B, 0X11D, 0X11E, 0X120, 0X122, 0X123, 0X125, 0X127, 0X128, 0X12A,
          0X12C, 0X12E, 0X12F, 0X131, 0X133, 0X134, 0X136, 0X138, 0X13A, 0X13B,
          0X13D, 0X13F, 0X140, 0X142, 0X144, 0X146, 0X147, 0X149, 0X14B, 0X14D,
          0X14E, 0X150, 0X152, 0X154, 0X155, 0X157, 0X159, 0X15B, 0X15C, 0X15E,
          0X160, 0X162, 0X163, 0X165, 0X167, 0X169, 0X16A, 0X16C, 0X16E, 0X170,
          0X171, 0X173, 0X175, 0X177, 0X179, 0X17A, 0X17C, 0X17E, 0X180, 0X182,
          0X183, 0X185, 0X187, 0X189, 0X18B, 0X18C, 0X18E, 0X190, 0X192, 0X194,
          0X195, 0X197, 0X199, 0X19B, 0X19D, 0X19E, 0X1A0, 0X1A2, 0X1A4, 0X1A6,
          0X1A8, 0X1A9, 0X1AB, 0X1AD, 0X1AF, 0X1B1, 0X1B3, 0X1B5, 0X1B6, 0X1B8,
          0X1BA, 0X1BC, 0X1BE, 0X1C0, 0X1C2, 0X1C4, 0X1C5, 0X1C7, 0X1C9, 0X1CB,
          0X1CD, 0X1CF, 0X1D1, 0X1D3, 0X1D5, 0X1D6, 0X1D8, 0X1DA, 0X1DC, 0X1DE,
          0X1E0, 0X1E2, 0X1E4, 0X1E6, 0X1E8, 0X1EA, 0X1EC, 0X1ED, 0X1EF, 0X1F1,
          0X1F3, 0X1F5, 0X1F7, 0X1F9, 0X1FB, 0X1FD, 0X1FF, 0X201, 0X203, 0X205,
          0X207, 0X209, 0X20B, 0X20D, 0X20F, 0X211, 0X213, 0X215, 0X217, 0X219,
          0X21B, 0X21D, 0X21F, 0X221, 0X223, 0X225, 0X227, 0X229, 0X22B, 0X22D,
          0X22F, 0X231, 0X233, 0X235, 0X237, 0X239, 0X23B, 0X23D, 0X23F, 0X242,
          0X244, 0X246, 0X248, 0X24A, 0X24C, 0X24E, 0X250, 0X252, 0X254, 0X256,
          0X259, 0X25B, 0X25D, 0X25F, 0X261, 0X263, 0X265, 0X267, 0X26A, 0X26C,
          0X26E, 0X270, 0X272, 0X274, 0X276, 0X279, 0X27B, 0X27D, 0X27F, 0X281,
          0X284, 0X286, 0X288, 0X28A, 0X28C, 0X28F, 0X291, 0X293, 0X295, 0X297,
          0X29A, 0X29C, 0X29E, 0X2A0, 0X2A3, 0X2A5, 0X2A7, 0X2A9, 0X2AC, 0X2AE,
          0X2B0, 0X2B3, 0X2B5, 0X2B7, 0X2B9, 0X2BC, 0X2BE, 0X2C0, 0X2C3, 0X2C5,
          0X2C7, 0X2CA, 0X2CC, 0X2CE, 0X2D1, 0X2D3, 0X2D5, 0X2D8, 0X2DA, 0X2DC,
          0X2DF, 0X2E1, 0X2E4, 0X2E6, 0X2E8, 0X2EB, 0X2ED, 0X2F0, 0X2F2, 0X2F5,
          0X2F7, 0X2F9, 0X2FC, 0X2FE, 0X301, 0X303, 0X306, 0X308, 0X30B, 0X30D,
          0X310, 0X312, 0X315, 0X317, 0X31A, 0X31C, 0X31F, 0X321, 0X324, 0X326,
          0X329, 0X32B, 0X32E, 0X330, 0X333, 0X336, 0X338, 0X33B, 0X33D, 0X340,
          0X343, 0X345, 0X348, 0X34B, 0X34D, 0X350, 0X352, 0X355, 0X358, 0X35A,
          0X35D, 0X360, 0X362, 0X365, 0X368, 0X36B, 0X36D, 0X370, 0X373, 0X376,
          0X378, 0X37B, 0X37E, 0X381, 0X383, 0X386, 0X389, 0X38C, 0X38F, 0X391,
          0X394, 0X397, 0X39A, 0X39D, 0X3A0, 0X3A2, 0X3A5, 0X3A8, 0X3AB, 0X3AE,
          0X3B1, 0X3B4, 0X3B7, 0X3BA, 0X3BD, 0X3C0, 0X3C3, 0X3C5, 0X3C8, 0X3CB,
          0X3CE, 0X3D1, 0X3D4, 0X3D7, 0X3DA, 0X3DE, 0X3E1, 0X3E4, 0X3E7, 0X3EA,
          0X3ED, 0X3F0, 0X3F3, 0X3F6, 0X3F9, 0X3FC, 0X3FF, 0X403, 0X406, 0X409,
          0X40C, 0X40F, 0X413, 0X416, 0X419, 0X41C, 0X41F, 0X423, 0X426, 0X429,
          0X42C, 0X430, 0X433, 0X436, 0X43A, 0X43D, 0X440, 0X444, 0X447, 0X44A,
          0X44E, 0X451, 0X455, 0X458, 0X45B, 0X45F, 0X462, 0X466, 0X469, 0X46D,
          0X470, 0X474, 0X477, 0X47B, 0X47E, 0X482, 0X486, 0X489, 0X48D, 0X490,
          0X494, 0X498, 0X49B, 0X49F, 0X4A3, 0X4A6, 0X4AA, 0X4AE, 0X4B1, 0X4B5,
          0X4B9, 0X4BD, 0X4C0, 0X4C4, 0X4C8, 0X4CC, 0X4D0, 0X4D4, 0X4D7, 0X4DB,
          0X4DF, 0X4E3, 0X4E7, 0X4EB, 0X4EF, 0X4F3, 0X4F7, 0X4FB, 0X4FF, 0X503,
          0X507, 0X50B, 0X50F, 0X513, 0X517, 0X51B, 0X520, 0X524, 0X528, 0X52C,
          0X530, 0X535, 0X539, 0X53D, 0X541, 0X546, 0X54A, 0X54E, 0X553, 0X557,
          0X55B, 0X560, 0X564, 0X569, 0X56D, 0X572, 0X576, 0X57B, 0X57F, 0X584,
          0X588, 0X58D, 0X591, 0X596, 0X59B, 0X59F, 0X5A4, 0X5A9, 0X5AD, 0X5B2,
          0X5B7, 0X5BC, 0X5C1, 0X5C5, 0X5CA, 0X5CF, 0X5D4, 0X5D9, 0X5DE, 0X5E3,
          0X5E8, 0X5ED, 0X5F2, 0X5F7, 0X5FC, 0X601, 0X606, 0X60B, 0X611, 0X616,
          0X61B, 0X620, 0X626, 0X62B, 0X630, 0X635, 0X63B, 0X640, 0X646, 0X64B,
          0X651, 0X656, 0X65C, 0X661, 0X667, 0X66C, 0X672, 0X678, 0X67D, 0X683,
          0X689, 0X68F, 0X694, 0X69A, 0X6A0, 0X6A6, 0X6AC, 0X6B2, 0X6B8, 0X6BE,
          0X6C4, 0X6CA, 0X6D0, 0X6D6, 0X6DC, 0X6E3, 0X6E9, 0X6EF, 0X6F6, 0X6FC,
          0X702, 0X709, 0X70F, 0X716, 0X71C, 0X723, 0X729, 0X730, 0X737, 0X73D,
          0X744, 0X74B, 0X752, 0X758, 0X75F, 0X766, 0X76D, 0X774, 0X77B, 0X782,
          0X789, 0X791, 0X798, 0X79F, 0X7A6, 0X7AE, 0X7B5, 0X7BD, 0X7C4, 0X7CC,
          0X7D3, 0X7DB, 0X7E2, 0X7EA, 0X7F2, 0X7FA, 0X801, 0X809, 0X811, 0X819,
          0X821, 0X829, 0X832, 0X83A, 0X842, 0X84A, 0X853, 0X85B, 0X863, 0X86C,
          0X875, 0X87D, 0X886, 0X88F, 0X897, 0X8A0, 0X8A9, 0X8B2, 0X8BB, 0X8C4,
          0X8CD, 0X8D7, 0X8E0, 0X8E9, 0X8F3, 0X8FC, 0X906, 0X90F, 0X919, 0X923,
          0X92C, 0X936, 0X940, 0X94A, 0X954, 0X95E, 0X969, 0X973, 0X97D, 0X988,
          0X992, 0X99D, 0X9A8, 0X9B2, 0X9BD, 0X9C8, 0X9D3, 0X9DE, 0X9E9, 0X9F5,
          0XA00, 0XA0C, 0XA17, 0XA23, 0XA2E, 0XA3A, 0XA46, 0XA52, 0XA5E, 0XA6A,
          0XA77, 0XA83, 0XA8F, 0XA9C, 0XAA9, 0XAB5, 0XAC2, 0XACF, 0XADC, 0XAEA,
          0XAF7, 0XB04, 0XB12, 0XB20, 0XB2D, 0XB3B, 0XB49, 0XB57, 0XB66, 0XB74,
          0XB83, 0XB91, 0XBA0, 0XBAF, 0XBBE, 0XBCD, 0XBDC, 0XBEC, 0XBFC, 0XC0B,
          0XC1B, 0XC2B, 0XC3B, 0XC4C, 0XC5C, 0XC6D, 0XC7E, 0XC8F, 0XCA0, 0XCB1,
          0XCC3, 0XCD4, 0XCE6, 0XCF8, 0XD0A, 0XD1D, 0XD2F, 0XD42, 0XD55, 0XD68,
          0XD7B, 0XD8F, 0XDA3, 0XDB6, 0XDCB, 0XDDF, 0XDF4, 0XE08, 0XE1D, 0XE33,
          0XE48, 0XE5E, 0XE74, 0XE8A, 0XEA0, 0XEB7, 0XECE, 0XEE5, 0XEFD, 0XF15,
          0XF2D, 0XF45, 0XF5E, 0XF76, 0XF90, 0XFA9, 0XFC3, 0XFDD, 0XFF8, 0X1012,
          0X102D, 0X1049, 0X1065, 0X1081, 0X109D, 0X10BA, 0X10D7, 0X10F5,
          0X1113, 0X1131, 0X1150, 0X116F, 0X118F, 0X11AF, 0X11D0, 0X11F1,
          0X1212, 0X1234, 0X1256, 0X1279, 0X129C, 0X12C0, 0X12E4, 0X1309,
          0X132F, 0X1355, 0X137B, 0X13A2, 0X13CA, 0X13F3, 0X141B, 0X1445,
          0X146F, 0X149A, 0X14C6, 0X14F2, 0X151F, 0X154D, 0X157B, 0X15AB,
          0X15DB, 0X160C, 0X163D, 0X1670, 0X16A3, 0X16D8, 0X170D, 0X1743,
          0X177A, 0X17B3, 0X17EC, 0X1826, 0X1861, 0X189E, 0X18DC, 0X191A,
          0X195B, 0X199C, 0X19DE, 0X1A22, 0X1A68, 0X1AAF, 0X1AF7, 0X1B40,
          0X1B8C, 0X1BD9, 0X1C27, 0X1C77, 0X1CC9, 0X1D1D, 0X1D73, 0X1DCB,
          0X1E25, 0X1E80, 0X1EDE, 0X1F3F, 0X1FA1, 0X2006, 0X206E, 0X20D8,
          0X2145, 0X21B5, 0X2227, 0X229D, 0X2316, 0X2392, 0X2411, 0X2494,
          0X251B, 0X25A6, 0X2635, 0X26C8, 0X275F, 0X27FB, 0X289C, 0X2942,
          0X29EE, 0X2A9F, 0X2B56, 0X2C13, 0X2CD6, 0X2DA1, 0X2E73, 0X2F4C,
          0X302D, 0X3117, 0X320A, 0X3306, 0X340C, 0X351D, 0X363A, 0X3762,
          0X3897, 0X39DA, 0X3B2C, 0X3C8D, 0X3DFF, 0X3F84, 0X411B, 0X42C8,
          0X448B, 0X4666, 0X485C, 0X4A6E, 0X4C9F, 0X4EF3, 0X516B, 0X540D,
          0X56DB, 0X59DB, 0X5D12, 0X6085, 0X643D, 0X6840, 0X6C99, 0X7153,
          0X767B, 0X7C20, 0X8256, 0X8933, 0X90D4, 0X995A, 0XA2F1, 0XADCF,
          0XBA3A, 0XC88E, 0XD946, 0XED07, 0X104BD, 0X121B6, 0X145EE, 0X1747F,
          0X1B295, 0X20981, 0X28BE3, 0X3652F, 0X517C8, 0XA2F8C, 0X1F4000, };
//table_Angle= table_Angle&0xFFF;

void SinCos_Table(p_Ang_SinCos PV)
{
    switch (PV->table_Angle & U_0)
    //  0---4095�ĵ�Ƕȿ��Ʒ��Ķβ�ѯ90
    {
    case U0_90:      //
        PV->table_Sin = fSin[PV->table_Angle];
        PV->table_Cos = fSin[0x3FF - PV->table_Angle];
        break;

    case U90_180:
        PV->table_Sin = fSin[0x0800 - PV->table_Angle];
        PV->table_Cos = -fSin[PV->table_Angle - 0x3FF];
        break;

    case U180_270:
        PV->table_Sin = -fSin[PV->table_Angle - 0x0800];
        PV->table_Cos = -fSin[0x0c00 - PV->table_Angle];
        break;

    case U270_360:
        PV->table_Sin = -fSin[0x1000 - PV->table_Angle];
        PV->table_Cos = fSin[PV->table_Angle - 0x0c00];
        break;

    }
}

void Atan_Cale(p_IQAtan pV)  // 90�ȼ��㣬��ȡ�����Һ���
{
    int16 i = 0;
    if (pV->Alpha == 0)
    {
        if (pV->Beta == 0)
            i = 0;
        else
            i = 1023;
    }
    else
    {
        pV->IQTan = Abs(pV->Beta) / Abs(pV->Alpha);

        if (ftan[i + 512] <= pV->IQTan)
            i += 512;
        if (ftan[i + 256] <= pV->IQTan)
            i += 256;
        if (ftan[i + 128] <= pV->IQTan)
            i += 128;
        if (ftan[i + 64] <= pV->IQTan)
            i += 64;
        if (ftan[i + 32] <= pV->IQTan)
            i += 32;
        if (ftan[i + 16] <= pV->IQTan)
            i += 16;
        if (ftan[i + 8] <= pV->IQTan)
            i += 8;
        if (ftan[i + 4] <= pV->IQTan)
            i += 4;
        if (ftan[i + 2] <= pV->IQTan)
            i += 2;
        if (ftan[i + 1] <= pV->IQTan)
            i += 1;
    }
    if (pV->Beta > 0)
    {
        if (pV->Alpha > 0)
            pV->IQAngle = i;
        else
            pV->IQAngle = 2048 - i;
    }
    else
    {
        if (pV->Alpha > 0)
            pV->IQAngle = 4096 - i;
        else
            pV->IQAngle = i + 2048;
    }
    pV->IQAngle = (pV->IQAngle & IQAngle_Range);
}

_iq10 Limit_Sat(_iq10 Uint, _iq10 U_max, _iq10 U_min) //���Ƹ�ֵ����
{
    _iq10 Uout;
    if (Uint <= U_min)
        Uout = U_min;
    else if (Uint >= U_max)
        Uout = U_max;
    else
        Uout = Uint;
    return Uout;
}

void Grad_XieLv(p_GXieLv pV)    // б���ݶȼ��� ������һ���ݶ�ֵ�ӼӼ���
{
    if (pV->XieLv_X < (pV->XieLv_Y - pV->XieLv_Grad)) // XieLv_Grad ���ݶ�ֵʱ��
    {
        pV->XieLv_Y = pV->XieLv_Y - pV->XieLv_Grad;
    }
    else if (pV->XieLv_X > (pV->XieLv_Y + pV->XieLv_Grad)) // ���ݶ�ֵʱ��
    {
        pV->XieLv_Y = pV->XieLv_Y + pV->XieLv_Grad;
    }
    else
    {
        pV->XieLv_Y = pV->XieLv_X;
    }
}

Uint32 IQSqrt(Uint32 M) //������������������ԭ�����԰ٶ�
{
    Uint32 N, i, tmp, ttp;
    if (M == 0)
        return 0;
    N = 0;

    tmp = (M >> 30);

    M <<= 2;
    if (tmp > 1)
    {
        N++;
        tmp -= N;
    }
    for (i = 15; i > 0; i--)
    {
        N <<= 1;

        tmp <<= 2;
        tmp += (M >> 30);

        ttp = N;
        ttp = (ttp << 1) + 1;

        M <<= 2;
        if (tmp >= ttp)
        {
            tmp -= ttp;
            N++;
        }
    }
    return N;
}

void Delay_100ns(Uint32 t) // Delay_us
{
    Uint32 delay_count1, delay_count2;
    for (delay_count2 = 0; delay_count2 < t; delay_count2++)
        for (delay_count1 = 0; delay_count1 < Delay_1Bns; delay_count1++)
            ;
}

void Delay_us(Uint32 t) // Delay_us
{
    Uint32 delay_count1, delay_count2;
    for (delay_count2 = 0; delay_count2 < t; delay_count2++)
        for (delay_count1 = 0; delay_count1 < Delay_1us; delay_count1++)
            ;
}

void Delay_ms(Uint32 t) // Delay_us
{
    Uint32 delay_count1, delay_count2;
    for (delay_count2 = 0; delay_count2 < t; delay_count2++)
        for (delay_count1 = 0; delay_count1 < Delay_1ms; delay_count1++)
            ;
}

// USER CODE END