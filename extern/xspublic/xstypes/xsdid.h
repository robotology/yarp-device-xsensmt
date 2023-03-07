
//  Copyright (c) 2003-2022 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.    Redistributions of source code must retain the above copyright notice,
//      this list of conditions, and the following disclaimer.
//  
//  2.    Redistributions in binary form must reproduce the above copyright notice,
//      this list of conditions, and the following disclaimer in the documentation
//      and/or other materials provided with the distribution.
//  
//  3.    Neither the names of the copyright holders nor the names of their contributors
//      may be used to endorse or promote products derived from this software without
//      specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef XSDID_H
#define XSDID_H

// DID Type (high nibble)
#define XS_DID_TYPEL_MASK                0x000F0000
#define XS_DID_TYPEH_MASK                0x00F00000
#define XS_DID_GPL_MASK                    0x0F000000
#define XS_DID_GPH_MASK                    0xF0000000
#define XS_DID_TYPE_MASK                (XS_DID_TYPEH_MASK | XS_DID_TYPEL_MASK)
#define XS_DID_GP_MASK                    (XS_DID_GPH_MASK | XS_DID_GPL_MASK)
#define XS_DID_MK4TYPE_MASK                (XS_DID_TYPEH_MASK | XS_DID_GPL_MASK)
#define XS_DID_ID_MASK                    0x0000FFFF
#define XS_DID_FULLTYPE_MASK            0xFFFF0000

#define XS_DID_TYPEL_SHIFT                16
#define XS_DID_TYPEH_SHIFT                20
#define XS_DID_GPL_SHIFT                24
#define XS_DID_GPH_SHIFT                28

#define XS_DID_TYPEH_INTERNAL            0x00000000
#define XS_DID_TYPEH_AWINDAMASTER        0x00200000
#define XS_DID_TYPEH_MT_X0                0x00600000
#define XS_DID_TYPEH_MT_X00                0x00700000
#define XS_DID_TYPEH_MTX2_MTW2            0x00B00000
#define XS_DID_TYPEH_BODYPACK            0x00A00000
#define XS_DID_TYPEH_MT_3X0                0x00D00000
#define XS_DID_TYPEH_MT_X_MPU            0x00800000

#define XS_DID_TYPEL_STATION            0x00000000
#define XS_DID_TYPEL_DONGLE                0x00010000
#define XS_DID_TYPEL_OEM                0x00020000
#define XS_DID_TYPEL_SYNCSTATION        0x00030000
#define XS_DID_TYPEL_BUS_MASTER            0x00040000
#define XS_DID_TYPEL_DONGLE_ANT            0x00010000        //Note: this is the same range as XS_DID_TYPEL_DONGLE

#define XS_DID_TYPEL_RS232                0x00000000
#define XS_DID_TYPEL_RS422                0x00010000
#define XS_DID_TYPEL_RS485XM            0x00020000
#define XS_DID_TYPEL_RS485                0x00030000
#define XS_DID_TYPEL_WIRELESS            0x00040000
#define XS_DID_TYPEL_COMM_MASK            0x00070000
#define XS_DID_TYPEL_MULTI                0x00080000
#define XS_DID_TYPEL_MK5                0x00080000

#define XS_DID_GPL_1                    0x01000000
#define XS_DID_GPL_2                    0x02000000
#define XS_DID_GPL_3                    0x03000000
#define XS_DID_GPL_7                    0x07000000
#define XS_DID_GPL_8                    0x08000000
#define XS_DID_GPL_10                    0x01000000
#define XS_DID_GPL_20                    0x02000000
#define XS_DID_GPL_30                    0x03000000
#define XS_DID_GPL_100                    0x01000000
#define XS_DID_GPL_200                    0x02000000
#define XS_DID_GPL_300                    0x03000000
#define XS_DID_GPL_400                    0x04000000
#define XS_DID_GPL_500                    0x05000000
#define XS_DID_GPL_600                    0x06000000
#define XS_DID_GPL_700                    0x07000000
#define XS_DID_GPL_800                    0x08000000
#define XS_DID_GPL_900                    0x09000000
#define XS_DID_GPL_IMU                    0x01000000
#define XS_DID_GPL_VRU                    0x02000000
#define XS_DID_GPL_AHRS                    0x03000000
#define XS_DID_GPL_AWINDA2                0x01000000
#define XS_DID_GPL_AHRSGNSS                XS_DID_GPL_400
#define XS_DID_GPL_AHRSGNSSG            XS_DID_GPL_500
#define XS_DID_GPL_GNSSINS                XS_DID_GPL_600
#define XS_DID_GPL_GNSSINSG                XS_DID_GPL_700
#define XS_DID_GPL_GNSSINSRTK            XS_DID_GPL_800

#define XS_DID_TYPE_AWINDA                XS_DID_TYPEH_AWINDAMASTER
#define XS_DID_TYPE_AWINDA_STATION        (XS_DID_TYPEH_AWINDAMASTER | XS_DID_TYPEL_STATION)
#define XS_DID_TYPE_AWINDA_DONGLE        (XS_DID_TYPEH_AWINDAMASTER | XS_DID_TYPEL_DONGLE)
#define XS_DID_TYPE_AWINDA_DONGLE_ANT    (XS_DID_TYPEH_AWINDAMASTER | XS_DID_TYPEL_DONGLE_ANT)
#define XS_DID_TYPE_AWINDA_OEM            (XS_DID_TYPEH_AWINDAMASTER | XS_DID_TYPEL_OEM)
#define XS_DID_TYPE_SYNCSTATION            (XS_DID_TYPEH_AWINDAMASTER | XS_DID_TYPEL_SYNCSTATION)

#define XS_DID_FULLTYPE_AWINDA2_STATION    (XS_DID_GPL_AWINDA2 | XS_DID_TYPE_AWINDA_STATION)
#define XS_DID_FULLTYPE_AWINDA2_DONGLE    (XS_DID_GPL_AWINDA2 | XS_DID_TYPE_AWINDA_DONGLE)
#define XS_DID_FULLTYPE_AWINDA2_OEM        (XS_DID_GPL_AWINDA2 | XS_DID_TYPE_AWINDA_OEM)
#define XS_DID_FULLTYPE_SYNCSTATION2    (XS_DID_GPL_AWINDA2 | XS_DID_TYPE_SYNCSTATION)

#define XS_DID_TYPE_MTX2                (XS_DID_TYPEH_MTX2_MTW2 | XS_DID_TYPEL_RS485XM)
#define XS_DID_TYPE_MTW2                (XS_DID_TYPEH_MTX2_MTW2 | XS_DID_TYPEL_WIRELESS)

#define XS_DID_MK4TYPE_MT_1_MPU            (XS_DID_TYPEH_MT_X_MPU | XS_DID_GPL_1)
#define XS_DID_MK4TYPE_MT_2_MPU            (XS_DID_TYPEH_MT_X_MPU | XS_DID_GPL_2)
#define XS_DID_MK4TYPE_MT_3_MPU            (XS_DID_TYPEH_MT_X_MPU | XS_DID_GPL_3)
#define XS_DID_MK4TYPE_MT_7_MPU            (XS_DID_TYPEH_MT_X_MPU | XS_DID_GPL_7)
#define XS_DID_MK4TYPE_MT_8_MPU            (XS_DID_TYPEH_MT_X_MPU | XS_DID_GPL_8)
#define XS_DID_MK4TYPE_MT_310            (XS_DID_TYPEH_MT_3X0 | XS_DID_GPL_1)
#define XS_DID_MK4TYPE_MT_320            (XS_DID_TYPEH_MT_3X0 | XS_DID_GPL_2)
#define XS_DID_MK4TYPE_MT_330            (XS_DID_TYPEH_MT_3X0 | XS_DID_GPL_3)
#define XS_DID_MK4TYPE_MT_10            (XS_DID_TYPEH_MT_X0 | XS_DID_GPL_10)
#define XS_DID_MK4TYPE_MT_20            (XS_DID_TYPEH_MT_X0 | XS_DID_GPL_20)
#define XS_DID_MK4TYPE_MT_30            (XS_DID_TYPEH_MT_X0 | XS_DID_GPL_30)
#define XS_DID_MK4TYPE_MT_100            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_100)
#define XS_DID_MK4TYPE_MT_200            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_200)
#define XS_DID_MK4TYPE_MT_300            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_300)
#define XS_DID_MK4TYPE_MT_400            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_400)
#define XS_DID_MK4TYPE_MT_500            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_500)
#define XS_DID_MK4TYPE_MT_600            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_600)
#define XS_DID_MK4TYPE_MT_700            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_700)
#define XS_DID_MK4TYPE_MT_800            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_800)
#define XS_DID_MK4TYPE_MT_900            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_900)

#define XS_DID_MK5TYPE_MT_10            (XS_DID_TYPEH_MT_X0 | XS_DID_GPL_10 | XS_DID_TYPEL_MK5)
#define XS_DID_MK5TYPE_MT_20            (XS_DID_TYPEH_MT_X0 | XS_DID_GPL_20 | XS_DID_TYPEL_MK5)
#define XS_DID_MK5TYPE_MT_30            (XS_DID_TYPEH_MT_X0 | XS_DID_GPL_30 | XS_DID_TYPEL_MK5)
#define XS_DID_MK5TYPE_MT_100            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_100 | XS_DID_TYPEL_MK5)
#define XS_DID_MK5TYPE_MT_200            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_200 | XS_DID_TYPEL_MK5)
#define XS_DID_MK5TYPE_MT_300            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_300 | XS_DID_TYPEL_MK5)
#define XS_DID_MK5TYPE_MT_400            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_400 | XS_DID_TYPEL_MK5)
#define XS_DID_MK5TYPE_MT_500            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_500 | XS_DID_TYPEL_MK5)
#define XS_DID_MK5TYPE_MT_600            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_600 | XS_DID_TYPEL_MK5)
#define XS_DID_MK5TYPE_MT_700            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_700 | XS_DID_TYPEL_MK5)
#define XS_DID_MK5TYPE_MT_800            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_800 | XS_DID_TYPEL_MK5)
#define XS_DID_MK5TYPE_MT_900            (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_900 | XS_DID_TYPEL_MK5)

#define XS_DID_MASTER                    0x00000000
#define XS_DID_ABMCLOCKMASTER            0x00000100
#define XS_DID_GLOVEMASTER                (XS_DID_TYPEH_INTERNAL | 0x00000200)

#define XS_DID_GLOVETYPE_UNKNOWN        0x00000000
#define XS_DID_GLOVETYPE_LEFT            0x00000001
#define XS_DID_GLOVETYPE_RIGHT            0x00000002

#define XS_DID_GLOVEMASTER_UNKNOWN        ( XS_DID_GLOVEMASTER | XS_DID_GLOVETYPE_UNKNOWN )
#define XS_DID_GLOVEMASTER_LEFT            ( XS_DID_GLOVEMASTER | XS_DID_GLOVETYPE_LEFT )
#define XS_DID_GLOVEMASTER_RIGHT        ( XS_DID_GLOVEMASTER | XS_DID_GLOVETYPE_RIGHT )

#define XS_DID_INTERNAL(did)            ((did & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_INTERNAL)
#define XS_DID_WM(did)                    ((did & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_AWINDAMASTER)
#define XS_DID_NOWM(did)                ((did & XS_DID_TYPEH_MASK) != XS_DID_TYPEH_AWINDAMASTER)
#define XS_DID_MTW2(did)                ((did & XS_DID_TYPE_MASK) == XS_DID_TYPE_MTW2)
#define XS_DID_MTX2(did)                ((did & XS_DID_TYPE_MASK) == XS_DID_TYPE_MTX2)

#define XS_DID_AWINDA2_STATION(did)        ((did & XS_DID_FULLTYPE_MASK) == XS_DID_FULLTYPE_AWINDA2_STATION)
#define XS_DID_AWINDA2_DONGLE(did)        ((did & XS_DID_FULLTYPE_MASK) == XS_DID_FULLTYPE_AWINDA2_DONGLE)
#define XS_DID_AWINDA2_OEM(did)            ((did & XS_DID_FULLTYPE_MASK) == XS_DID_FULLTYPE_AWINDA2_OEM)

#define XS_DID_SYNCSTATION2(did)        ((did & XS_DID_FULLTYPE_MASK) == XS_DID_FULLTYPE_SYNCSTATION2)
#define XS_DID_SYNCSTATION(did)            (XS_DID_SYNCSTATION2(did))

#define XS_DID_AWINDA2(did)                (XS_DID_AWINDA2_STATION(did) || XS_DID_AWINDA2_DONGLE(did) || XS_DID_AWINDA2_OEM(did))

#define XS_DID_BODYPACK(did)            ((did & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_BODYPACK)

#define XS_DID_MK4TYPE_MT_710_RANGE_START        (XS_DID_TYPEH_MT_X00 | XS_DID_GPL_700 | 0x1000)
#define XS_DID_MK5TYPE_RANGE_START        0x00002000

#define XS_DID64_BIT                    0x0000000080000000ULL

#endif
