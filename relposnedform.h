/*
    relposnedform.h (part of GNSS-Stylus)
    Copyright (C) 2019-2021 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

/**
 * @file relposnedform.h
 * @brief Declaration for a form that shows contents of RELPOSNED-message
 */

#ifndef RELPOSNEDFORM_H
#define RELPOSNEDFORM_H

#include <QWidget>
#include <QTreeWidgetItem>

#include "gnssmessage.h"

namespace Ui {
class RELPOSNEDForm;
}

/**
 * @brief Form used to show contents of RELPOSNED-message.
 */
class RELPOSNEDForm : public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief Constructor
     * @param parent Parent widget
     * @param title Window title
     */
    explicit RELPOSNEDForm(QWidget *parent = nullptr, const QString& title = "RELPOSNED");
    ~RELPOSNEDForm();

    /**
     * @brief Updates window's fields
     * @param relposnedMessage RELPOSNED-message
     */
    void updateFields(const UBXMessage_RELPOSNED& relposnedMessage);

protected:
    void showEvent(QShowEvent* event);  //!< Initializes some things that can't be initialized in constructor

private:
    Ui::RELPOSNEDForm *ui;

    QTreeWidgetItem *treeItem_version;
    QTreeWidgetItem *treeItem_refStationId;
    QTreeWidgetItem *treeItem_iTOW;
    QTreeWidgetItem *treeItem_relPosN;
    QTreeWidgetItem *treeItem_relPosE;
    QTreeWidgetItem *treeItem_relPosD;
    QTreeWidgetItem *treeItem_relPosLength;
    QTreeWidgetItem *treeItem_relPosHeading;
    QTreeWidgetItem *treeItem_accN;
    QTreeWidgetItem *treeItem_accE;
    QTreeWidgetItem *treeItem_accD;
    QTreeWidgetItem *treeItem_accLength;
    QTreeWidgetItem *treeItem_accHeading;
    QTreeWidgetItem *treeItem_flags;
    QTreeWidgetItem *treeItem_flag_gnssFixOK;
    QTreeWidgetItem *treeItem_flag_diffSoln;
    QTreeWidgetItem *treeItem_flag_relPosValid;
    QTreeWidgetItem *treeItem_flag_carrSoln;
    QTreeWidgetItem *treeItem_flag_isMoving;
    QTreeWidgetItem *treeItem_flag_refPosMiss;
    QTreeWidgetItem *treeItem_flag_refObsMiss;
    QTreeWidgetItem *treeItem_flag_relPosHeadingValid;

    bool treeItemsCreated;

};

#endif // RELPOSNEDFORM_H
