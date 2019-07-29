#include "ntripclientform.h"
#include "ui_ntripclientform.h"

NTRIPClientForm::NTRIPClientForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::NTRIPClientForm)
{
    ui->setupUi(this);
}

NTRIPClientForm::~NTRIPClientForm()
{
    delete ui;
}
