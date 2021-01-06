/*
    relposnedform.cpp (part of GNSS-Stylus)
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

#include "relposnedform.h"
#include "ui_relposnedform.h"

RELPOSNEDForm::RELPOSNEDForm(QWidget *parent, const QString& title) :
    QWidget(parent),
    ui(new Ui::RELPOSNEDForm)
{
    ui->setupUi(this);
    treeItemsCreated = false;
    this->setWindowTitle(title);
}

RELPOSNEDForm::~RELPOSNEDForm()
{
    delete ui;
}

void RELPOSNEDForm::showEvent(QShowEvent* event)
{
    QWidget::showEvent(event);

    UBXMessage_RELPOSNED relposned;
    updateFields(relposned);
}

void RELPOSNEDForm::updateFields(const UBXMessage_RELPOSNED& relposnedMessage)
{
    if (!treeItemsCreated)
    {
        // Seems that Qt doesn't allow creating of QTreeWidgetItems
        // inside constructor (caused segmentation fault).
        // So create them here at first round.

        treeItem_version = new QTreeWidgetItem(ui->treeWidget);
        treeItem_refStationId = new QTreeWidgetItem(ui->treeWidget);
        treeItem_iTOW = new QTreeWidgetItem(ui->treeWidget);

        treeItem_relPosN = new QTreeWidgetItem(ui->treeWidget);
        treeItem_relPosE = new QTreeWidgetItem(ui->treeWidget);
        treeItem_relPosD = new QTreeWidgetItem(ui->treeWidget);
        treeItem_relPosLength = new QTreeWidgetItem(ui->treeWidget);
        treeItem_relPosHeading = new QTreeWidgetItem(ui->treeWidget);

        treeItem_accN = new QTreeWidgetItem(ui->treeWidget);
        treeItem_accE = new QTreeWidgetItem(ui->treeWidget);
        treeItem_accD = new QTreeWidgetItem(ui->treeWidget);
        treeItem_accLength = new QTreeWidgetItem(ui->treeWidget);
        treeItem_accHeading = new QTreeWidgetItem(ui->treeWidget);

        treeItem_flags = new QTreeWidgetItem(ui->treeWidget);
        treeItem_flag_gnssFixOK = new QTreeWidgetItem(ui->treeWidget);
        treeItem_flag_diffSoln = new QTreeWidgetItem(ui->treeWidget);
        treeItem_flag_relPosValid = new QTreeWidgetItem(ui->treeWidget);
        treeItem_flag_carrSoln = new QTreeWidgetItem(ui->treeWidget);
        treeItem_flag_isMoving = new QTreeWidgetItem(ui->treeWidget);
        treeItem_flag_refPosMiss = new QTreeWidgetItem(ui->treeWidget);
        treeItem_flag_refObsMiss = new QTreeWidgetItem(ui->treeWidget);
        treeItem_flag_relPosHeadingValid = new QTreeWidgetItem(ui->treeWidget);

        treeItem_version->setText(0, "version");
        treeItem_refStationId->setText(0, "refStationId");
        treeItem_iTOW->setText(0, "iTOW");

        treeItem_relPosN->setText(0, "relPosN");
        treeItem_relPosE->setText(0, "relPosE");
        treeItem_relPosD->setText(0, "relPosD");
        treeItem_relPosLength->setText(0, "relPosLength");
        treeItem_relPosHeading->setText(0, "relPosHeading");

        treeItem_accN->setText(0, "accN");
        treeItem_accE->setText(0, "accE");
        treeItem_accD->setText(0, "accD");
        treeItem_accLength->setText(0, "accLength");
        treeItem_accHeading->setText(0, "accHeading");

        treeItem_flags->setText(0, "flags");
        treeItem_flag_gnssFixOK->setText(0, "flag_gnssFixOK");
        treeItem_flag_diffSoln->setText(0, "flag_diffSoln");
        treeItem_flag_relPosValid->setText(0, "flag_relPosValid");
        treeItem_flag_carrSoln->setText(0, "flag_carrSoln");
        treeItem_flag_isMoving->setText(0, "flag_isMoving");
        treeItem_flag_refPosMiss->setText(0, "flag_refPosMiss");
        treeItem_flag_refObsMiss->setText(0, "flag_refObsMiss");
        treeItem_flag_relPosHeadingValid->setText(0, "flag_relPosHeadingValid");

        treeItemsCreated = true;

        for (int i=0;i<ui->treeWidget->topLevelItemCount();i++)
        {
            ui->treeWidget->topLevelItem(i)->setTextAlignment(0, Qt::AlignRight);
        }
    }

    treeItem_version->setText(1, QString::number(relposnedMessage.version));
    treeItem_refStationId->setText(1, QString::number(relposnedMessage.refStationId));
    treeItem_iTOW->setText(1, QString::number(relposnedMessage.iTOW));
    treeItem_relPosN->setText(1, QString::number(relposnedMessage.relPosN, 'f', 3));
    treeItem_relPosE->setText(1, QString::number(relposnedMessage.relPosE, 'f', 3));
    treeItem_relPosD->setText(1, QString::number(relposnedMessage.relPosD, 'f', 3));
    treeItem_relPosLength->setText(1, QString::number(relposnedMessage.relPosLength, 'f', 3));
    treeItem_relPosHeading->setText(1, QString::number(relposnedMessage.relPosHeading, 'f', 3));
    treeItem_accN->setText(1, QString::number(relposnedMessage.accN, 'f', 3));
    treeItem_accE->setText(1, QString::number(relposnedMessage.accE, 'f', 3));
    treeItem_accD->setText(1, QString::number(relposnedMessage.accD, 'f', 3));
    treeItem_accLength->setText(1, QString::number(relposnedMessage.accLength, 'f', 3));
    treeItem_accHeading->setText(1, QString::number(relposnedMessage.accHeading, 'f', 3));

    treeItem_flags->setText(1, QString::number(relposnedMessage.flags));
    treeItem_flag_gnssFixOK->setText(1, QString::number(relposnedMessage.flag_gnssFixOK));
    treeItem_flag_diffSoln->setText(1, QString::number(relposnedMessage.flag_diffSoln));
    treeItem_flag_relPosValid->setText(1, QString::number(relposnedMessage.flag_relPosValid));
    // Handled below treeItem_flag_carrSoln->setText(1, QString::number(relposnedMessage.flag_carrSoln));
    treeItem_flag_isMoving->setText(1, QString::number(relposnedMessage.flag_isMoving));
    treeItem_flag_refPosMiss->setText(1, QString::number(relposnedMessage.flag_refPosMiss));
    treeItem_flag_refObsMiss->setText(1, QString::number(relposnedMessage.flag_refObsMiss));
    treeItem_flag_relPosHeadingValid->setText(1, QString::number(relposnedMessage.flag_relPosHeadingValid));

    const QBrush okBrush(QColor(128,255,128));
    const QBrush errorBrush(QColor(255,128,128));
    const QBrush warningBrush(QColor(255,255,0));

    if (relposnedMessage.flag_gnssFixOK)
    {
        treeItem_flag_gnssFixOK->setBackground(1, okBrush);
    }
    else
    {
        treeItem_flag_gnssFixOK->setBackground(1, errorBrush);
    }

    if (relposnedMessage.flag_diffSoln)
    {
        treeItem_flag_diffSoln->setBackground(1, okBrush);
    }
    else
    {
        treeItem_flag_diffSoln->setBackground(1, errorBrush);
    }

    if (relposnedMessage.flag_relPosValid)
    {
        treeItem_flag_relPosValid->setBackground(1, okBrush);
    }
    else
    {
        treeItem_flag_relPosValid->setBackground(1, errorBrush);
    }

    if (relposnedMessage.flag_carrSoln == UBXMessage_RELPOSNED::NO_SOLUTION)
    {
        treeItem_flag_carrSoln->setText(1, "0 (No sol)");
        treeItem_flag_carrSoln->setBackground(1, errorBrush);
    }
    else if (relposnedMessage.flag_carrSoln == UBXMessage_RELPOSNED::FLOATING)
    {
        treeItem_flag_carrSoln->setText(1, "1 (Float)");
        treeItem_flag_carrSoln->setBackground(1, warningBrush);
    }
    else if (relposnedMessage.flag_carrSoln == UBXMessage_RELPOSNED::FIXED)
    {
        treeItem_flag_carrSoln->setText(1, "2 (Fixed)");
        treeItem_flag_carrSoln->setBackground(1, okBrush);
    }
    else
    {
        treeItem_flag_carrSoln->setText(1, "3 (Error)");
        treeItem_flag_carrSoln->setBackground(1, errorBrush);
    }

    if (!relposnedMessage.flag_isMoving)
    {
        treeItem_flag_isMoving->setBackground(1, okBrush);
    }
    else
    {
        treeItem_flag_isMoving->setBackground(1, warningBrush);
    }

    if (!relposnedMessage.flag_refPosMiss)
    {
        treeItem_flag_refPosMiss->setBackground(1, okBrush);
    }
    else
    {
        treeItem_flag_refPosMiss->setBackground(1, warningBrush);
    }

    if (!relposnedMessage.flag_refObsMiss)
    {
        treeItem_flag_refObsMiss->setBackground(1, okBrush);
    }
    else
    {
        treeItem_flag_refObsMiss->setBackground(1, warningBrush);
    }

    if (relposnedMessage.flag_relPosHeadingValid)
    {
        treeItem_flag_relPosHeadingValid->setBackground(1, okBrush);
    }
    else
    {
        treeItem_flag_relPosHeadingValid->setBackground(1, warningBrush);
    }

}
