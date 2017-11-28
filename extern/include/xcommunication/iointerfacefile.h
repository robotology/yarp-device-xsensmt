/*	Copyright (c) 2003-2017 Xsens Technologies B.V. or subsidiaries worldwide.
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1.	Redistributions of source code must retain the above copyright notice,
		this list of conditions and the following disclaimer.

	2.	Redistributions in binary form must reproduce the above copyright notice,
		this list of conditions and the following disclaimer in the documentation
		and/or other materials provided with the distribution.

	3.	Neither the names of the copyright holders nor the names of their contributors
		may be used to endorse or promote products derived from this software without
		specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
	THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
	SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
	OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
	TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef IOINTERFACEFILE_H
#define IOINTERFACEFILE_H

#include <xsens/xsplatform.h>
#include "streaminterface.h"
#include <xsens/xsresultvalue.h>

/*! \brief The low-level file communication class.
*/
class IoInterfaceFile : public IoInterface
{
private:
	XSENS_DISABLE_COPY(IoInterfaceFile)

protected:
		//! The file handle, also indicates if the file is open or not.
	XsFile* m_handle;
		//! Contains the size of the file
	XsFilePos m_fileSize;
		//! The last read position in the file
	XsFilePos m_readPos;
		//! The last write position in the file
	XsFilePos m_writePos;
		//! The last result of an operation
	mutable XsResultValue m_lastResult;
		//! Contains the name of the file that was last successfully opened.
	XsString m_filename;
	/*! \brief Indicates whether the last operation was a read or write operation.

		This value is used to check whether or not a seek is required to perform a
		requested read or write operation.
	*/
	bool m_reading;
		//! Indicates if the file was opened in read-only mode
	bool m_readOnly;

	void gotoRead();
	void gotoWrite();
public:
	IoInterfaceFile();
	~IoInterfaceFile();

	// Function overrides
	XsResultValue close() override;
	XsResultValue closeFile();
	XsResultValue flushData() override;
	bool isOpen() const override;
	XsResultValue getLastResult() const override;
	XsResultValue writeData(const XsByteArray& data, XsSize *written = nullptr) override;
	XsResultValue readData(XsSize maxLength, XsByteArray& data) override;
	XsResultValue readTerminatedData(XsSize maxLength, unsigned char terminator, XsByteArray& bdata);

	// Other functions
	XsResultValue appendData(const XsByteArray& bdata);
	XsResultValue closeAndDelete();
	XsResultValue create(const XsString& filename);
	XsResultValue deleteData(XsFilePos start, XsSize length);
	XsResultValue find(const XsByteArray& data, XsFilePos& pos);
	XsFilePos getFileSize() const;
	XsTimeStamp getFileDate() const;
	XsResultValue getName(XsString& filename) const;
	XsString getFileName() const;
	XsFilePos getReadPosition() const;
	XsFilePos getWritePosition() const;
	XsResultValue insertData(XsFilePos start, const XsByteArray& data);
	bool isReadOnly() const;
	XsResultValue open(const XsString& filename, bool createNew, bool readOnly);
	XsResultValue setReadPosition(XsFilePos pos);
	XsResultValue setWritePosition(XsFilePos pos = -1);
	XsResultValue reserve(XsFilePos minSize);
};

#endif
