#ifndef NTRIPCLIENTFORM_H
#define NTRIPCLIENTFORM_H

#include <QWidget>

namespace Ui {
class NTRIPClientForm;
}

class NTRIPClientForm : public QWidget
{
    Q_OBJECT

public:
    explicit NTRIPClientForm(QWidget *parent = nullptr);
    ~NTRIPClientForm();

private:
    Ui::NTRIPClientForm *ui;
};

#endif // NTRIPCLIENTFORM_H
