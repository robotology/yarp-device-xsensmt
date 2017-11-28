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

//lint -e1512 -e1509 -e1511 -e534 -e578 -e1060 -e613
#include "datapacket_p.h"

/*! \cond XS_INTERNAL */

/*! \class DataPacketPrivate
	\brief Internal administration for contained data of XsDataPacket class.
	\details This is only the part that can be stored in an XsMessage, so no TOA and computed packet IDs
*/

/*! \brief Copy constructor */
DataPacketPrivate::DataPacketPrivate(DataPacketPrivate const& p)
	: MapType()	// start with clean map
{
	*this = p;
}

/*! \brief Destructor */
DataPacketPrivate::~DataPacketPrivate()
{
	try {
		clear();
	} catch(...)
	{
	}
}

/*! \brief Assignment operator */
const DataPacketPrivate& DataPacketPrivate::operator = (const DataPacketPrivate& p)
{
	if (this != &p)
	{
		clear();
		for (auto i : p)
			insert(i.first, i.second->clone());
	}
	return *this;
}

/*! \brief Clear the contents */
void DataPacketPrivate::clear()
{
	for (auto it : *this)
		delete it.second;
	MapType::clear();
}

/*! \brief Find the item matching \a id
	\details This function will return an iterator to the item that matches \a id or end() if it could not find it.
	The function uses a loose comparison to ensure entry ambiguity. As a result, the returned iterator may have a
	slightly different id than the supplied one.
	\return An iterator to the requested item or end()
*/
MapType::const_iterator DataPacketPrivate::find(XsDataIdentifier id) const
{
	return MapType::find(id & XDI_FullTypeMask);
}

/*! \brief Add or overwrite the item with \a id
	\details The function will create a new item or overwite the current item, properly cleaning up existing data
	if necessary.
*/
MapType::iterator DataPacketPrivate::insert(XsDataIdentifier id, XsDataPacket_Private::Variant* var)
{
	id = id & XDI_FullTypeMask;
	auto it = MapType::lower_bound(id);
	if (it != end() && it->first == id)
	{
		delete it->second;
		it->second = var;
		return it;
	}
	else
		return MapType::insert(it, std::make_pair(id & XDI_FullTypeMask, var));
}

/*! \brief Remove the item with \a id if it exists, cleaning up associated data */
void DataPacketPrivate::erase(XsDataIdentifier id)
{
	auto it = find(id);
	if (it != end())
		erase(it);
}

/*! \brief Remove the item at \a it, cleaning up associated data */
void DataPacketPrivate::erase(MapType::const_iterator it)
{
	delete it->second;
	MapType::erase(it);
}

/*! \brief Merge \a other into this
	\details The function will copy all items from \a other into this. Existing items will only be overwritten if
	\a overwrite is set to true. This function does not detect conflicts between closely related but IDs (such as
	frame range and packet counter). This is left to the caller.
*/
void DataPacketPrivate::merge(DataPacketPrivate const& other, bool overwrite)
{
	if (overwrite)
		for (auto i : other)
			insert(i.first, i.second->clone());
	else
	{
		for (auto i : other)
		{
			auto j = find(i.first);
			if (j == end())
				insert(i.first, i.second->clone());
		}
	}
}

/*! \endcond XS_INTERNAL */
