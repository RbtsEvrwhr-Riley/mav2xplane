#include "mainwindow.h"
#include <QApplication>
#include <QCoreApplication>
#include <map>
#include <QDebug>
#include <iostream>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

   /* std::multimap<float,int> q;


    q.insert(std::pair<float,int>(6.,8));
    q.insert ( std::pair<float,int>(3.5,100));
    q.insert ( std::pair<float,int>(3.5,6100));


    std::map<float,int>::iterator it1 = q.begin();
    for(it1=q.begin(); it1 != q.end(); ++it1)
        qDebug() << it1->first << " " << it1->second;*/
	QCoreApplication::setOrganizationName("Robots Everywhere");
    QCoreApplication::setOrganizationDomain("robots-everywhere.com");
    QCoreApplication::setApplicationName("Mav2Xplane");



    return a.exec();
}
