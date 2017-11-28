/*	WARNING: COPYRIGHT (C) 2017 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE. ALL RIGHTS RESERVED.
	THIS FILE AND THE SOURCE CODE IT CONTAINS (AND/OR THE BINARY CODE FILES FOUND IN THE SAME
	FOLDER THAT CONTAINS THIS FILE) AND ALL RELATED SOFTWARE (COLLECTIVELY, "CODE") ARE SUBJECT
	TO A RESTRICTED LICENSE AGREEMENT ("AGREEMENT") BETWEEN XSENS AS LICENSOR AND THE AUTHORIZED
	LICENSEE UNDER THE AGREEMENT. THE CODE MUST BE USED SOLELY WITH XSENS PRODUCTS INCORPORATED
	INTO LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY USE, MODIFICATION, COPYING OR
	DISTRIBUTION OF THE CODE IS STRICTLY PROHIBITED UNLESS EXPRESSLY AUTHORIZED BY THE AGREEMENT.
	IF YOU ARE NOT AN AUTHORIZED USER OF THE CODE IN ACCORDANCE WITH THE AGREEMENT, YOU MUST STOP
	USING OR VIEWING THE CODE NOW, REMOVE ANY COPIES OF THE CODE FROM YOUR COMPUTER AND NOTIFY
	XSENS IMMEDIATELY BY EMAIL TO INFO@XSENS.COM. ANY COPIES OR DERIVATIVES OF THE CODE (IN WHOLE
	OR IN PART) IN SOURCE CODE FORM THAT ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE
	COPYRIGHT NOTICE AND THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
*/

#ifndef DATAPACKET_P_H
#define DATAPACKET_P_H

#include "xsmessage.h"
#include "xsdeviceid.h"
#include "xstimestamp.h"
#include <map>
#include "xsquaternion.h"
#include "xsushortvector.h"
#include "xsvector3.h"
#include "xsscrdata.h"
#include "xstriggerindicationdata.h"
#include "xseuler.h"
#include "xsmatrix3x3.h"
#include "xsrange.h"
#include "xsutctime.h"
#include "xsrawgnsspvtdata.h"
#include "xsrawgnsssatinfo.h"
#include "xsrawgpsdop.h"
#include "xsrawgpssol.h"
#include "xsrawgpssvinfo.h"
#include "xsrawgpstimeutc.h"
#include "xsbytearray.h"

#include "xssnapshot.h"

/*! \cond XS_INTERNAL */
namespace XsDataPacket_Private {
	class Variant {
	public:
		Variant(XsDataIdentifier id) : m_id(id) {}
		virtual ~Variant() {}
		virtual void readFromMessage(XsMessage const& msg, int offset, int dSize) = 0;
		virtual void writeToMessage(XsMessage& msg, int offset) const = 0;
		virtual int sizeInMsg() const = 0;
		virtual Variant* clone() const = 0;

		void setDataId(XsDataIdentifier id)
		{
			assert((m_id & XDI_FullTypeMask) == (id & XDI_FullTypeMask));
			m_id = id;
		}
		XsDataIdentifier dataId() const { return m_id; }

		template <typename U>
		U& toDerived()
		{
			U* ptr = dynamic_cast<U*>(this);
			assert(ptr);
			return *ptr;
		}

		template <typename U>
		U const& toDerived() const
		{
			U const* ptr = dynamic_cast<U const*>(this);
			assert(ptr);
			return *ptr;
		}

	private:
		XsDataIdentifier m_id;
	};

	template <typename T, int C = 1>
	class GenericVariant : public Variant {
	public:
		GenericVariant(XsDataIdentifier id) : Variant(id) {}

		void readFromMessage(XsMessage const& msg, int offset, int /*ignored*/) override
		{
			msg.getData<T>(data(), dataId(), offset, C);
		}
		void writeToMessage(XsMessage& msg, int offset) const override
		{
			msg.setData<T>(constData(), dataId(), offset, C);
		}
		virtual T* data() = 0;
		virtual T const* constData() const = 0;

		int sizeInMsg() const override
		{
			return XsMessage::sizeInMsg<T>(dataId(), C);
		}
	};

	template<>
	inline void GenericVariant<uint64_t, 1>::readFromMessage(XsMessage const& msg, int offset, int /*ignored*/)
	{
		*data() = 0;
		*data() = ((uint64_t)XsMessage_getDataLong(&msg, offset)) << 32;
		*data() += ((uint64_t)XsMessage_getDataLong(&msg, offset + 4));
	}

	template<>
	inline void GenericVariant<uint64_t, 1>::writeToMessage(XsMessage& msg, int offset) const
	{
		XsMessage_setDataLong(&msg, (uint32_t) ((*constData()) >> 32), offset);
		XsMessage_setDataLong(&msg, (uint32_t) ((*constData()) & 0xFFFFFFFF), offset);
	}

	template <typename T>
	struct SimpleVariant : public GenericVariant<T, 1> {
		using GenericVariant<T, 1>::dataId;

		SimpleVariant(XsDataIdentifier id) : GenericVariant<T, 1>(id), m_data() {}
		SimpleVariant(XsDataIdentifier id, T const& val) : GenericVariant<T, 1>(id), m_data(val) {}

		T m_data;
		T* data() override
		{
			return &m_data;
		}
		T const* constData() const override
		{
			return &m_data;
		}

		Variant* clone() const override
		{
			return new SimpleVariant<T>(dataId(), m_data);
		}

		int sizeInMsg() const override
		{
			return XsMessage::sizeInMsg<T>(dataId(), 1);
		}
	};

	template <typename U, typename T, int C>
	struct ComplexVariant : public GenericVariant<T, C>
	{
		using GenericVariant<T, C>::dataId;
		ComplexVariant(XsDataIdentifier id) : GenericVariant<T, C>(id) {}
		ComplexVariant(XsDataIdentifier id, U const& val) : GenericVariant<T, C>(id), m_data(val) {}

		U m_data;
		T* data() override
		{
			return const_cast<T*>(m_data.data());
		}
		T const* constData() const override
		{
			return m_data.data();
		}

		//! \note Override in derived classes!
		Variant* clone() const override
		{
			return new ComplexVariant<U, T, C>(dataId(), m_data);
		}
	};

	struct XsQuaternionVariant : public ComplexVariant<XsQuaternion, XsReal, 4>
	{
		XsQuaternionVariant(XsDataIdentifier id) : ComplexVariant<XsQuaternion, XsReal, 4>(id) {}
		XsQuaternionVariant(XsDataIdentifier id, XsQuaternion const& val) : ComplexVariant<XsQuaternion, XsReal, 4>(id, val) {}

		Variant* clone() const override
		{
			return new XsQuaternionVariant(dataId(), m_data);
		}
	};

	struct XsUShortVectorVariant : public ComplexVariant<XsUShortVector, uint16_t, 3>
	{
		XsUShortVectorVariant(XsDataIdentifier id) : ComplexVariant<XsUShortVector, unsigned short, 3>(id) {}
		XsUShortVectorVariant(XsDataIdentifier id, XsUShortVector const& val) : ComplexVariant<XsUShortVector, unsigned short, 3>(id, val) {}
		Variant* clone() const override
		{
			return new XsUShortVectorVariant(dataId(), m_data);
		}
	};

	struct XsVector3Variant : public ComplexVariant<XsVector3, XsReal, 3>
	{
		XsVector3Variant(XsDataIdentifier id) : ComplexVariant<XsVector3, XsReal, 3>(id) {}
		XsVector3Variant(XsDataIdentifier id, XsVector const& val) : ComplexVariant<XsVector3, XsReal, 3>(id, val)
		{
			assert(val.size() == 3);
		}
		Variant* clone() const override
		{
			return new XsVector3Variant(dataId(), m_data);
		}
	};

	struct XsVector2Variant : public ComplexVariant<XsVector, XsReal, 2>
	{
		XsVector2Variant(XsDataIdentifier id) : ComplexVariant<XsVector, XsReal, 2>(id, XsVector(2,0)) {}
		XsVector2Variant(XsDataIdentifier id, XsVector const& val) : ComplexVariant<XsVector, XsReal, 2>(id, val)
		{
			assert(val.size() == 2);
		}
		Variant* clone() const override
		{
			return new XsVector2Variant(dataId(), m_data);
		}
	};

	struct XsScrDataVariant : public Variant
	{
		XsScrDataVariant(XsDataIdentifier id) : Variant(id) {}
		XsScrDataVariant(XsDataIdentifier id, XsScrData const& val) : Variant(id), m_data(val) {}
		Variant* clone() const override
		{
			return new XsScrDataVariant(dataId(), m_data);
		}

		XsScrData m_data;
		void readFromMessage(XsMessage const& msg, int offset, int /*ignored*/) override
		{
			for (int i = 0; i < 3; ++i, offset +=2)
				m_data.m_acc[i] = msg.getDataShort(offset);
			for (int i = 0; i < 3; ++i, offset +=2)
				m_data.m_gyr[i] = msg.getDataShort(offset);
			for (int i = 0; i < 3; ++i, offset +=2)
				m_data.m_mag[i] = msg.getDataShort(offset);
			m_data.m_temp = msg.getDataShort(offset);
		}
		void writeToMessage(XsMessage& msg, int offset) const override
		{
			for (int i = 0; i < 3; ++i, offset +=2)
				msg.setDataShort(m_data.m_acc[i], offset);
			for (int i = 0; i < 3; ++i, offset +=2)
				msg.setDataShort(m_data.m_gyr[i], offset);
			for (int i = 0; i < 3; ++i, offset +=2)
				msg.setDataShort(m_data.m_mag[i], offset);
			msg.setDataShort(m_data.m_temp, offset);
		}

		int sizeInMsg() const override
		{
			return 10*sizeof(uint16_t);
		}
	};

	struct XsTriggerIndicationDataVariant : public Variant
	{
		XsTriggerIndicationDataVariant(XsDataIdentifier id) : Variant(id) {}
		XsTriggerIndicationDataVariant(XsDataIdentifier id, XsTriggerIndicationData const& val) : Variant(id), m_data(val) {}
		Variant* clone() const override
		{
			return new XsTriggerIndicationDataVariant(dataId(), m_data);
		}

		XsTriggerIndicationData m_data;
		void readFromMessage(XsMessage const& msg, int offset, int /*ignored*/) override
		{
			m_data.m_line        = XsMessage_getDataByte (&msg, offset + 0);
			m_data.m_polarity    = XsMessage_getDataByte (&msg, offset + 1);
			m_data.m_timestamp   = XsMessage_getDataLong (&msg, offset + 2);
			m_data.m_frameNumber = XsMessage_getDataShort(&msg, offset + 6);
		}
		void writeToMessage(XsMessage& msg, int offset) const override
		{
			XsMessage_setDataByte (&msg, m_data.m_line,        offset + 0);
			XsMessage_setDataByte (&msg, m_data.m_polarity,    offset + 1);
			XsMessage_setDataLong (&msg, m_data.m_timestamp,   offset + 2);
			XsMessage_setDataShort(&msg, m_data.m_frameNumber, offset + 6);
		}

		int sizeInMsg() const override
		{
			return 8;
		}
	};

	struct XsEulerVariant : public ComplexVariant<XsEuler, XsReal, 3>
	{
		XsEulerVariant(XsDataIdentifier id) : ComplexVariant<XsEuler, XsReal, 3>(id) {}
		XsEulerVariant(XsDataIdentifier id, XsEuler const& val) : ComplexVariant<XsEuler, XsReal, 3>(id, val) {}
		Variant* clone() const override
		{
			return new XsEulerVariant(dataId(), m_data);
		}
	};

	struct XsMatrixVariant : public ComplexVariant<XsMatrix3x3, XsReal, 9>
	{
		XsMatrixVariant(XsDataIdentifier id) : ComplexVariant<XsMatrix3x3, XsReal, 9>(id) {}
		XsMatrixVariant(XsDataIdentifier id, XsMatrix const& val) : ComplexVariant<XsMatrix3x3, XsReal, 9>(id, val) {}
		Variant* clone() const override
		{
			return new XsMatrixVariant(dataId(), m_data);
		}

		void readFromMessage(XsMessage const& msg, int offset, int /*ignored*/) override
		{
			int ds = XsMessage_getFPValueSize(dataId());
			int k = 0;
			for (int i=0 ; i<3 ; ++i)
				for (int j=0 ; j<3 ; ++j, k+=ds)
					XsMessage_getDataFPValuesById(&msg, dataId(), &m_data[j][i], offset+k, 1);
		}
		void writeToMessage(XsMessage& msg, int offset) const override
		{
			int ds = XsMessage_getFPValueSize(dataId());
			int k = 0;
			for (int i=0 ; i<3 ; ++i)
				for (int j=0 ; j<3 ; ++j, k+=ds)
					XsMessage_setDataFPValuesById(&msg, dataId(), &m_data[j][i], offset+k, 1);
		}
	};

	struct XsRangeVariant : public Variant
	{
		XsRangeVariant(XsDataIdentifier id) : Variant(id) {}
		XsRangeVariant(XsDataIdentifier id, XsRange const& val) : Variant(id), m_data(val) {}
		Variant* clone() const override
		{
			return new XsRangeVariant(dataId(), m_data);
		}

		XsRange m_data;
		void readFromMessage(XsMessage const& msg, int offset, int /*ignored*/) override
		{
			// unwrap
			uint16_t first = (uint16_t) XsMessage_getDataShort(&msg, offset + 0);
			uint16_t last = (uint16_t) XsMessage_getDataShort(&msg, offset + 2);
			m_data.setRange(first, (int)((uint16_t)(last - first)) + (int)first);
		}
		void writeToMessage(XsMessage& msg, int offset) const override
		{
			XsMessage_setDataShort(&msg, (uint16_t) m_data.first(), offset + 0);
			XsMessage_setDataShort(&msg, (uint16_t) m_data.last(), offset + 2);
		}

		int sizeInMsg() const override
		{
			return 4;
		}
	};

	struct XsUtcTimeVariant : public Variant
	{
		XsUtcTimeVariant(XsDataIdentifier id) : Variant(id), m_data() {}
		XsUtcTimeVariant(XsDataIdentifier id, XsUtcTime const& val) : Variant(id), m_data(val) {}
		Variant* clone() const override
		{
			return new XsUtcTimeVariant(dataId(), m_data);
		}

		XsUtcTime m_data;
		void readFromMessage(XsMessage const& msg, int offset, int /*ignored*/) override
		{
			m_data.m_nano = XsMessage_getDataLong(&msg, offset);
			m_data.m_year = XsMessage_getDataShort(&msg, offset+4);

			//lint --e{662, 661} these fields are named first-of-array fields
			// month, day, hour, minute, second and valid
			uint8_t* bareByte = (uint8_t*) &m_data.m_month;
			for (int i=0; i < 6; ++i)
				bareByte[i] = XsMessage_getDataByte(&msg, offset + 6 + i);
		}
		void writeToMessage(XsMessage& msg, int offset) const override
		{
			// update
			XsMessage_setDataLong(&msg, m_data.m_nano, offset);
			XsMessage_setDataShort(&msg, m_data.m_year, offset + 4);

			// month, day, hour, minute, second and valid
			uint8_t* bareByte = (uint8_t*) &m_data.m_month;
			for (int i=0; i<6;++i)
				XsMessage_setDataByte(&msg, bareByte[i], offset + 6 + i);	//lint !e661 !e662
		}

		int sizeInMsg() const override
		{
			return 12;
		}
	};

	struct XsRawGnssPvtDataVariant : public Variant
	{
		XsRawGnssPvtDataVariant(XsDataIdentifier id) : Variant(id), m_data() {}
		XsRawGnssPvtDataVariant(XsDataIdentifier id, XsRawGnssPvtData const& val) : Variant(id), m_data(val) {}
		Variant* clone() const override
		{
			return new XsRawGnssPvtDataVariant(dataId(), m_data);
		}

		XsRawGnssPvtData m_data;
		void readFromMessage(XsMessage const& msg, int offset, int /*ignored*/) override
		{
			m_data.m_itow    = XsMessage_getDataLong(&msg, offset + 0);
			m_data.m_year   = XsMessage_getDataShort(&msg, offset + 4);
			m_data.m_month   = XsMessage_getDataByte(&msg, offset + 6);
			m_data.m_day     = XsMessage_getDataByte(&msg, offset + 7);
			m_data.m_hour    = XsMessage_getDataByte(&msg, offset + 8);
			m_data.m_min     = XsMessage_getDataByte(&msg, offset + 9);
			m_data.m_sec     = XsMessage_getDataByte(&msg, offset + 10);
			m_data.m_valid   = XsMessage_getDataByte(&msg, offset + 11);
			m_data.m_tAcc    = XsMessage_getDataLong(&msg, offset + 12);
			m_data.m_nano    = XsMessage_getDataLong(&msg, offset + 16);
			m_data.m_fixType = XsMessage_getDataByte(&msg, offset + 20);
			m_data.m_flags   = XsMessage_getDataByte(&msg, offset + 21);
			m_data.m_numSv   = XsMessage_getDataByte(&msg, offset + 22);
			m_data.m_res1    = XsMessage_getDataByte(&msg, offset + 23);
			m_data.m_lon     = XsMessage_getDataLong(&msg, offset + 24);
			m_data.m_lat     = XsMessage_getDataLong(&msg, offset + 28);
			m_data.m_height  = XsMessage_getDataLong(&msg, offset + 32);
			m_data.m_hMsl    = XsMessage_getDataLong(&msg, offset + 36);
			m_data.m_hAcc    = XsMessage_getDataLong(&msg, offset + 40);
			m_data.m_vAcc    = XsMessage_getDataLong(&msg, offset + 44);
			m_data.m_velN    = XsMessage_getDataLong(&msg, offset + 48);
			m_data.m_velE    = XsMessage_getDataLong(&msg, offset + 52);
			m_data.m_velD    = XsMessage_getDataLong(&msg, offset + 56);
			m_data.m_gSpeed  = XsMessage_getDataLong(&msg, offset + 60);
			m_data.m_headMot = XsMessage_getDataLong(&msg, offset + 64);
			m_data.m_sAcc    = XsMessage_getDataLong(&msg, offset + 68);
			m_data.m_headAcc = XsMessage_getDataLong(&msg, offset + 72);
			m_data.m_headVeh = XsMessage_getDataLong(&msg, offset + 76);
			m_data.m_gdop   = XsMessage_getDataShort(&msg, offset + 80);
			m_data.m_pdop   = XsMessage_getDataShort(&msg, offset + 82);
			m_data.m_tdop   = XsMessage_getDataShort(&msg, offset + 84);
			m_data.m_vdop   = XsMessage_getDataShort(&msg, offset + 86);
			m_data.m_hdop   = XsMessage_getDataShort(&msg, offset + 88);
			m_data.m_ndop   = XsMessage_getDataShort(&msg, offset + 90);
			m_data.m_edop   = XsMessage_getDataShort(&msg, offset + 92);
		}
		void writeToMessage(XsMessage& msg, int offset) const override
		{
			XsMessage_setDataLong (&msg, m_data.m_itow   , offset + 0);
			XsMessage_setDataShort(&msg, m_data.m_year   , offset + 4);
			XsMessage_setDataByte (&msg, m_data.m_month  , offset + 6);
			XsMessage_setDataByte (&msg, m_data.m_day    , offset + 7);
			XsMessage_setDataByte (&msg, m_data.m_hour   , offset + 8);
			XsMessage_setDataByte (&msg, m_data.m_min    , offset + 9);
			XsMessage_setDataByte (&msg, m_data.m_sec    , offset + 10);
			XsMessage_setDataByte (&msg, m_data.m_valid  , offset + 11);
			XsMessage_setDataLong (&msg, m_data.m_tAcc   , offset + 12);
			XsMessage_setDataLong (&msg, m_data.m_nano   , offset + 16);
			XsMessage_setDataByte (&msg, m_data.m_fixType, offset + 20);
			XsMessage_setDataByte (&msg, m_data.m_flags  , offset + 21);
			XsMessage_setDataByte (&msg, m_data.m_numSv  , offset + 22);
			XsMessage_setDataByte (&msg, m_data.m_res1   , offset + 23);
			XsMessage_setDataLong (&msg, m_data.m_lon    , offset + 24);
			XsMessage_setDataLong (&msg, m_data.m_lat    , offset + 28);
			XsMessage_setDataLong (&msg, m_data.m_height , offset + 32);
			XsMessage_setDataLong (&msg, m_data.m_hMsl   , offset + 36);
			XsMessage_setDataLong (&msg, m_data.m_hAcc   , offset + 40);
			XsMessage_setDataLong (&msg, m_data.m_vAcc   , offset + 44);
			XsMessage_setDataLong (&msg, m_data.m_velN   , offset + 48);
			XsMessage_setDataLong (&msg, m_data.m_velE   , offset + 52);
			XsMessage_setDataLong (&msg, m_data.m_velD   , offset + 56);
			XsMessage_setDataLong (&msg, m_data.m_gSpeed , offset + 60);
			XsMessage_setDataLong (&msg, m_data.m_headMot, offset + 64);
			XsMessage_setDataLong (&msg, m_data.m_sAcc   , offset + 68);
			XsMessage_setDataLong (&msg, m_data.m_headAcc, offset + 72);
			XsMessage_setDataLong (&msg, m_data.m_headVeh, offset + 76);
			XsMessage_setDataShort(&msg, m_data.m_gdop   , offset + 80);
			XsMessage_setDataShort(&msg, m_data.m_pdop   , offset + 82);
			XsMessage_setDataShort(&msg, m_data.m_tdop   , offset + 84);
			XsMessage_setDataShort(&msg, m_data.m_vdop   , offset + 86);
			XsMessage_setDataShort(&msg, m_data.m_hdop   , offset + 88);
			XsMessage_setDataShort(&msg, m_data.m_ndop   , offset + 90);
			XsMessage_setDataShort(&msg, m_data.m_edop   , offset + 92);
		}

		int sizeInMsg() const override
		{
			return 94;
		}
	};

	struct XsRawGnssSatInfoVariant : public Variant
	{
		XsRawGnssSatInfoVariant(XsDataIdentifier id) : Variant(id), m_data() {}
		XsRawGnssSatInfoVariant(XsDataIdentifier id, XsRawGnssSatInfo const& val) : Variant(id), m_data(val) {}
		Variant* clone() const override
		{
			return new XsRawGnssSatInfoVariant(dataId(), m_data);
		}

		XsRawGnssSatInfo m_data;
		void readFromMessage(XsMessage const& msg, int offset, int /*ignored*/) override
		{
			m_data.m_itow   = XsMessage_getDataLong(&msg, offset + 0);
			m_data.m_numSvs = XsMessage_getDataByte(&msg, offset + 4);
			m_data.m_res1   = XsMessage_getDataByte(&msg, offset + 5);
			m_data.m_res2   = XsMessage_getDataByte(&msg, offset + 6);
			m_data.m_res3   = XsMessage_getDataByte(&msg, offset + 7);

			offset = offset + 8;
			for (uint8_t i = 0; i < m_data.m_numSvs; ++i)
			{
				m_data.m_satInfos[i].m_gnssId = XsMessage_getDataByte(&msg, offset + 0);
				m_data.m_satInfos[i].m_svId   = XsMessage_getDataByte(&msg, offset + 1);
				m_data.m_satInfos[i].m_cno    = XsMessage_getDataByte(&msg, offset + 2);
				m_data.m_satInfos[i].m_flags  = XsMessage_getDataByte(&msg, offset + 3);
				offset += 4;
			}
		}
		void writeToMessage(XsMessage& msg, int offset) const override
		{
			XsMessage_setDataLong(&msg, m_data.m_itow   , offset + 0);
			XsMessage_setDataByte(&msg, m_data.m_numSvs , offset + 4);
			XsMessage_setDataByte(&msg, m_data.m_res1   , offset + 5);
			XsMessage_setDataByte(&msg, m_data.m_res2   , offset + 6);
			XsMessage_setDataByte(&msg, m_data.m_res3   , offset + 7);

			offset = offset + 8;
			for (uint8_t i = 0; i < m_data.m_numSvs; ++i)
			{
				XsMessage_setDataByte(&msg, m_data.m_satInfos[i].m_gnssId , offset + 0);
				XsMessage_setDataByte(&msg, m_data.m_satInfos[i].m_svId   , offset + 1);
				XsMessage_setDataByte(&msg, m_data.m_satInfos[i].m_cno    , offset + 2);
				XsMessage_setDataByte(&msg, m_data.m_satInfos[i].m_flags  , offset + 3);
				offset += 4;
			}
		}

		int sizeInMsg() const override
		{
			return 8 + 4*m_data.m_numSvs;
		}
	};

	struct XsRawGpsDopVariant : public Variant
	{
		XsRawGpsDopVariant(XsDataIdentifier id) : Variant(id), m_data() {}
		XsRawGpsDopVariant(XsDataIdentifier id, XsRawGpsDop const& val) : Variant(id), m_data(val) {}
		Variant* clone() const override
		{
			return new XsRawGpsDopVariant(dataId(), m_data);
		}

		XsRawGpsDop m_data;
		void readFromMessage(XsMessage const& msg, int offset, int /*ignored*/) override
		{
			m_data.m_itow = XsMessage_getDataLong (&msg, offset);
			m_data.m_gdop = XsMessage_getDataShort(&msg, offset+4);
			m_data.m_pdop = XsMessage_getDataShort(&msg, offset+6);
			m_data.m_tdop = XsMessage_getDataShort(&msg, offset+8);
			m_data.m_vdop = XsMessage_getDataShort(&msg, offset+10);
			m_data.m_hdop = XsMessage_getDataShort(&msg, offset+12);
			m_data.m_ndop = XsMessage_getDataShort(&msg, offset+14);
			m_data.m_edop = XsMessage_getDataShort(&msg, offset+16);
		}
		void writeToMessage(XsMessage& msg, int offset) const override
		{
			XsMessage_setDataLong (&msg, m_data.m_itow, offset);
			XsMessage_setDataShort(&msg, m_data.m_gdop, offset+4);
			XsMessage_setDataShort(&msg, m_data.m_pdop, offset+6);
			XsMessage_setDataShort(&msg, m_data.m_tdop, offset+8);
			XsMessage_setDataShort(&msg, m_data.m_vdop, offset+10);
			XsMessage_setDataShort(&msg, m_data.m_hdop, offset+12);
			XsMessage_setDataShort(&msg, m_data.m_ndop, offset+14);
			XsMessage_setDataShort(&msg, m_data.m_edop, offset+16);
		}

		int sizeInMsg() const override
		{
			return 18;
		}
	};

	struct XsRawGpsSolVariant : public Variant
	{
		XsRawGpsSolVariant(XsDataIdentifier id) : Variant(id), m_data() {}
		XsRawGpsSolVariant(XsDataIdentifier id, XsRawGpsSol const& val) : Variant(id), m_data(val) {}
		Variant* clone() const override
		{
			return new XsRawGpsSolVariant(dataId(), m_data);
		}

		XsRawGpsSol m_data;
		void readFromMessage(XsMessage const& msg, int offset, int /*ignored*/) override
		{
			m_data.m_itow	= XsMessage_getDataLong (&msg, offset);
			m_data.m_frac	= XsMessage_getDataLong (&msg, offset+4);
			m_data.m_week	= XsMessage_getDataShort(&msg, offset+8);
			m_data.m_gpsfix = XsMessage_getDataByte (&msg, offset+10);
			m_data.m_flags	= XsMessage_getDataByte (&msg, offset+11);
			m_data.m_ecef_x = XsMessage_getDataLong (&msg, offset+12);
			m_data.m_ecef_y = XsMessage_getDataLong (&msg, offset+16);
			m_data.m_ecef_z = XsMessage_getDataLong (&msg, offset+20);
			m_data.m_pacc	= XsMessage_getDataLong (&msg, offset+24);
			m_data.m_ecef_vx= XsMessage_getDataLong (&msg, offset+28);
			m_data.m_ecef_vy= XsMessage_getDataLong (&msg, offset+32);
			m_data.m_ecef_vz= XsMessage_getDataLong (&msg, offset+36);
			m_data.m_sacc	= XsMessage_getDataLong (&msg, offset+40);
			m_data.m_pdop	= XsMessage_getDataShort(&msg, offset+44);
			m_data.m_res1	= XsMessage_getDataByte (&msg, offset+46);
			m_data.m_numsv	= XsMessage_getDataByte (&msg, offset+47);
			m_data.m_res2	= XsMessage_getDataLong (&msg, offset+48);
		}
		void writeToMessage(XsMessage& msg, int offset) const override
		{
			XsMessage_setDataLong (&msg, m_data.m_itow	, offset);
			XsMessage_setDataLong (&msg, m_data.m_frac	, offset+4);
			XsMessage_setDataShort(&msg, m_data.m_week	, offset+8);
			XsMessage_setDataByte (&msg, m_data.m_gpsfix  , offset+10);
			XsMessage_setDataByte (&msg, m_data.m_flags	, offset+11);
			XsMessage_setDataLong (&msg, m_data.m_ecef_x  , offset+12);
			XsMessage_setDataLong (&msg, m_data.m_ecef_y  , offset+16);
			XsMessage_setDataLong (&msg, m_data.m_ecef_z  , offset+20);
			XsMessage_setDataLong (&msg, m_data.m_pacc	, offset+24);
			XsMessage_setDataLong (&msg, m_data.m_ecef_vx , offset+28);
			XsMessage_setDataLong (&msg, m_data.m_ecef_vy , offset+32);
			XsMessage_setDataLong (&msg, m_data.m_ecef_vz , offset+36);
			XsMessage_setDataLong (&msg, m_data.m_sacc	, offset+40);
			XsMessage_setDataShort(&msg, m_data.m_pdop	, offset+44);
			XsMessage_setDataByte (&msg, m_data.m_res1	, offset+46);
			XsMessage_setDataByte (&msg, m_data.m_numsv	, offset+47);
			XsMessage_setDataLong (&msg, m_data.m_res2	, offset+48);
		}

		int sizeInMsg() const override
		{
			return 52;
		}
	};

	struct XsRawGpsTimeUtcVariant : public Variant
	{
		XsRawGpsTimeUtcVariant(XsDataIdentifier id) : Variant(id), m_data() {}
		XsRawGpsTimeUtcVariant(XsDataIdentifier id, XsRawGpsTimeUtc const& val) : Variant(id), m_data(val) {}
		Variant* clone() const override
		{
			return new XsRawGpsTimeUtcVariant(dataId(), m_data);
		}

		XsRawGpsTimeUtc m_data;
		void readFromMessage(XsMessage const& msg, int offset, int /*ignored*/) override
		{
			m_data.m_itow  = XsMessage_getDataLong (&msg, offset);
			m_data.m_tacc  = XsMessage_getDataLong (&msg, offset+4);
			m_data.m_nano  = XsMessage_getDataLong (&msg, offset+8);
			m_data.m_year  = XsMessage_getDataShort(&msg, offset+12);
			m_data.m_month = XsMessage_getDataByte (&msg, offset+14);
			m_data.m_day   = XsMessage_getDataByte (&msg, offset+15);
			m_data.m_hour  = XsMessage_getDataByte (&msg, offset+16);
			m_data.m_min   = XsMessage_getDataByte (&msg, offset+17);
			m_data.m_sec   = XsMessage_getDataByte (&msg, offset+18);
			m_data.m_valid = XsMessage_getDataByte (&msg, offset+19);
		}
		void writeToMessage(XsMessage& msg, int offset) const override
		{
			XsMessage_setDataLong (&msg, m_data.m_itow , offset);
			XsMessage_setDataLong (&msg, m_data.m_tacc , offset+4);
			XsMessage_setDataLong (&msg, m_data.m_nano , offset+8);
			XsMessage_setDataShort(&msg, m_data.m_year , offset+12);
			XsMessage_setDataByte (&msg, m_data.m_month, offset+14);
			XsMessage_setDataByte (&msg, m_data.m_day  , offset+15);
			XsMessage_setDataByte (&msg, m_data.m_hour , offset+16);
			XsMessage_setDataByte (&msg, m_data.m_min  , offset+17);
			XsMessage_setDataByte (&msg, m_data.m_sec  , offset+18);
			XsMessage_setDataByte (&msg, m_data.m_valid, offset+19);
		}

		int sizeInMsg() const override
		{
			return 20;
		}
	};

	struct XsRawGpsSvInfoVariant : public Variant
	{
		XsRawGpsSvInfoVariant(XsDataIdentifier id) : Variant(id), m_data() {}
		XsRawGpsSvInfoVariant(XsDataIdentifier id, XsRawGpsSvInfo const& val) : Variant(id), m_data(val) {}
		Variant* clone() const override
		{
			return new XsRawGpsSvInfoVariant(dataId(), m_data);
		}

		XsRawGpsSvInfo m_data;
		void readFromMessage(XsMessage const& msg, int offset, int /*ignored*/) override
		{
			m_data.m_itow = XsMessage_getDataLong(&msg, offset);
			m_data.m_nch = XsMessage_getDataByte( &msg, offset+4);
			m_data.m_res1 = XsMessage_getDataByte(&msg, offset+5);
			m_data.m_res2= XsMessage_getDataShort(&msg, offset+6);

			offset += 8;
			for (int i = 0; i < m_data.m_nch; ++i)
			{
				m_data.m_svInfos[i].m_chn   = XsMessage_getDataByte( &msg, offset+0);
				m_data.m_svInfos[i].m_svid  = XsMessage_getDataByte( &msg, offset+1);
				m_data.m_svInfos[i].m_flags = XsMessage_getDataByte( &msg, offset+2);
				m_data.m_svInfos[i].m_qi    = XsMessage_getDataByte( &msg, offset+3);
				m_data.m_svInfos[i].m_cno   = XsMessage_getDataByte( &msg, offset+4);
				m_data.m_svInfos[i].m_elev  = XsMessage_getDataByte( &msg, offset+5);
				m_data.m_svInfos[i].m_azim  = XsMessage_getDataShort(&msg, offset+6);
				m_data.m_svInfos[i].m_prres = XsMessage_getDataLong( &msg, offset+8);
				offset += 12;
			}
		}
		void writeToMessage(XsMessage& msg, int offset) const override
		{
			XsMessage_setDataLong(&msg, m_data.m_itow, offset);
			XsMessage_setDataByte( &msg, m_data.m_nch , offset+4);
			XsMessage_setDataByte(&msg, m_data.m_res1, offset+5);
			XsMessage_setDataShort(&msg, m_data.m_res2, offset+6);

			offset += 8;
			for (int i = 0; i < m_data.m_nch; ++i)
			{
				XsMessage_setDataByte( &msg, m_data.m_svInfos[i].m_chn  , offset+0);
				XsMessage_setDataByte( &msg, m_data.m_svInfos[i].m_svid , offset+1);
				XsMessage_setDataByte( &msg, m_data.m_svInfos[i].m_flags, offset+2);
				XsMessage_setDataByte( &msg, m_data.m_svInfos[i].m_qi   , offset+3);
				XsMessage_setDataByte( &msg, m_data.m_svInfos[i].m_cno  , offset+4);
				XsMessage_setDataByte( &msg, m_data.m_svInfos[i].m_elev , offset+5);
				XsMessage_setDataShort(&msg, m_data.m_svInfos[i].m_azim , offset+6);
				XsMessage_setDataLong( &msg, m_data.m_svInfos[i].m_prres, offset+8);
				offset += 12;
			}
		}

		int sizeInMsg() const override
		{
			return 8 + 12*m_data.m_nch;
		}
	};

	struct XsFullSnapshotVariant : public Variant
	{
		XsFullSnapshotVariant(XsDataIdentifier id) : Variant(id) {}

		XsFullSnapshotVariant(XsDataIdentifier id, XsSnapshot const& val) : Variant(id), m_data(val) {}

		Variant* clone() const override
		{
			return new XsFullSnapshotVariant(dataId(), m_data);
		}

		XsSnapshot m_data;
		void readFromMessage(XsMessage const& msg, int offset, int /*ignored*/) override
		{
			m_data.m_frameNumber = XsMessage_getDataShort(&msg, offset); offset += 2;
			m_data.m_timestamp = XsMessage_getDataLongLong(&msg, offset); offset += 8;
			for (int i = 0; i < 4; ++i, offset += 4)
				m_data.m_iQ[i] = XsMessage_getDataLong(&msg, offset);
			for (int i = 0; i < 3; ++i, offset += 8)
				m_data.m_iV[i] = XsMessage_getDataLongLong(&msg, offset);
			for (int i = 0; i < 3; ++i, offset += 4)
				m_data.m_mag[i] = XsMessage_getDataLong(&msg, offset);

			m_data.m_baro = XsMessage_getDataLong(&msg, offset); offset += 4;
			m_data.m_accClippingCounter = XsMessage_getDataByte(&msg, offset); offset += 1;
			m_data.m_gyrClippingCounter = XsMessage_getDataByte(&msg, offset); offset += 1;
			m_data.m_status = XsMessage_getDataShort(&msg, offset);
			m_data.m_type = ST_Full;
		}

		void writeToMessage(XsMessage& msg, int offset) const override
		{
			assert(m_data.m_type == ST_Full);
			XsMessage_setDataShort(&msg, m_data.m_frameNumber, offset);	offset += 2;
			XsMessage_setDataLongLong(&msg, m_data.m_timestamp, offset); offset += 8;
			for (int i = 0; i < 4; ++i, offset += 4)
				XsMessage_setDataLong(&msg, m_data.m_iQ[i], offset);
			for (int i = 0; i < 3; ++i, offset += 8)
				XsMessage_setDataLongLong(&msg, m_data.m_iV[i], offset);
			for (int i = 0; i < 3; ++i, offset += 4)
				XsMessage_setDataLong(&msg, m_data.m_mag[i], offset);

			XsMessage_setDataLong(&msg, m_data.m_baro, offset);	offset += 4;
			XsMessage_setDataByte(&msg, m_data.m_accClippingCounter, offset); offset += 1;
			XsMessage_setDataByte(&msg, m_data.m_gyrClippingCounter, offset); offset += 1;
			XsMessage_setDataShort(&msg, m_data.m_status, offset);
		}

		int sizeInMsg() const override
		{
			return 70;
		}
	};

	struct XsAwindaSnapshotVariant : public Variant
	{
		XsAwindaSnapshotVariant(XsDataIdentifier id) : Variant(id)
		{
			m_data.m_type = ST_Awinda;
		}
		XsAwindaSnapshotVariant(XsDataIdentifier id, XsSnapshot const& val) : Variant(id), m_data(val)
		{
			m_data.m_type = ST_Awinda;
		}
		Variant* clone() const override
		{
			return new XsAwindaSnapshotVariant(dataId(), m_data);
		}

		XsSnapshot m_data;
		void readFromMessage(XsMessage const& msg, int offset, int /*ignored*/) override
		{
			m_data.m_deviceId    = XsMessage_getDataLong(&msg, offset);	offset += 4;
			m_data.m_frameNumber = XsMessage_getDataLong(&msg, offset);	offset += 4;
			for (int i = 0; i < 3; ++i, offset += 4)
				m_data.m_iQ[i] = XsMessage_getDataLong(&msg, offset);
			for (int i = 0; i < 3; ++i, offset += 4)
				m_data.m_iV[i] = (int64_t) (int32_t) XsMessage_getDataLong(&msg, offset);
			for (int i = 0; i < 3; ++i, offset += 2)
				m_data.m_mag[i] = (int32_t)(int16_t)XsMessage_getDataShort(&msg, offset);
			m_data.m_baro = XsMessage_getDataLong(&msg, offset);	offset += 4;
			m_data.m_status = XsMessage_getDataShort(&msg, offset);	offset += 2;
			m_data.m_accClippingCounter = XsMessage_getDataByte(&msg, offset);	offset += 1;
			m_data.m_gyrClippingCounter = XsMessage_getDataByte(&msg, offset);
			m_data.m_type = ST_Awinda;
		}
		void writeToMessage(XsMessage& msg, int offset) const override
		{
			assert(m_data.m_type == ST_Awinda);
			XsMessage_setDataLong(&msg, m_data.m_deviceId.toInt(), offset);	offset += 4;
			XsMessage_setDataLong(&msg, m_data.m_frameNumber, offset);	offset += 4;
			for (int i = 0; i < 3; ++i, offset += 4)
				XsMessage_setDataLong(&msg, m_data.m_iQ[i], offset);
			for (int i = 0; i < 3; ++i, offset += 4)
				XsMessage_setDataLong(&msg, (uint32_t) (int32_t) m_data.m_iV[i], offset);
			for (int i = 0; i < 3; ++i, offset += 2)
				XsMessage_setDataShort(&msg, m_data.m_mag[i], offset);
			XsMessage_setDataLong(&msg, m_data.m_baro, offset);	offset += 4;
			XsMessage_setDataShort(&msg, m_data.m_status, offset);	offset += 2;
			XsMessage_setDataByte(&msg, m_data.m_accClippingCounter, offset);	offset += 1;
			XsMessage_setDataByte(&msg, m_data.m_gyrClippingCounter, offset);
		}

		int sizeInMsg() const override
		{
			return 46;
		}
	};

	struct XsByteArrayVariant : public Variant
	{
		XsByteArrayVariant(XsDataIdentifier id) : Variant(id)
		{
		}
		XsByteArrayVariant(XsDataIdentifier id, XsByteArray const& val) : Variant(id), m_data(val)
		{
		}
		Variant* clone() const override
		{
			return new XsByteArrayVariant(dataId(), m_data);
		}

		XsByteArray m_data;
		void readFromMessage(XsMessage const& msg, int offset, int dSize) override
		{
			m_data.assign(dSize, XsMessage_getDataBuffer(&msg, offset));
		}
		void writeToMessage(XsMessage& msg, int offset) const override
		{
			XsMessage_setDataBuffer(&msg, m_data.data(), m_data.size(), offset);
		}

		int sizeInMsg() const override
		{
			return (int) m_data.size();
		}
	};
}

typedef std::map<XsDataIdentifier, XsDataPacket_Private::Variant*> MapType;

struct DataPacketPrivate : private MapType {
	DataPacketPrivate() {}
	DataPacketPrivate(DataPacketPrivate const& p);
	~DataPacketPrivate();
	const DataPacketPrivate& operator = (const DataPacketPrivate& p);
	void erase(XsDataIdentifier id);
	void erase(MapType::const_iterator it);
	MapType::iterator insert(XsDataIdentifier id, XsDataPacket_Private::Variant* var);

	void clear();
	void merge(DataPacketPrivate const& other, bool overwrite);

	MapType::const_iterator find(XsDataIdentifier id) const;

	using MapType::begin;
	using MapType::end;
	using MapType::size;
	using MapType::empty;
};

/*! \endcond */

#endif
