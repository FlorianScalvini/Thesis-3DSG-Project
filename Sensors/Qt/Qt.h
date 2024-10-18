//
// Created by Florian on 08/12/22.
//

#ifndef OUTDOORNAV_QT_H
#define OUTDOORNAV_QT_H

#include <vector>
#include <iostream>
#include <string>
#include <QApplication>
#include <QDialog>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

class QtFrame
{
public:
    static int selectAttribut(std::vector<std::string> list)
    {
        // Create a null argc and argv for QApplication
        int argc = 0;
        char* argv[] = {nullptr};

        QApplication app(argc, argv);
        std::string selectedValue;
        QDialog dialog;
        dialog.setWindowTitle("Pop-up selection");
        int selectedIndex = -1; // Initialize to -1 (no selection)
        QVBoxLayout layout(&dialog);
        QListWidget listWidget;
        for(int i = 0; i < 5; i++)
            listWidget.addItem(list[i].c_str());

        layout.addWidget(&listWidget);

        QPushButton selectButton("Select");
        layout.addWidget(&selectButton);

        QObject::connect(&selectButton, &QPushButton::clicked, [&listWidget, &selectedIndex, &dialog]() {
            QListWidgetItem* selectedItem = listWidget.currentItem();
            if (selectedItem) {
                selectedIndex = listWidget.row(selectedItem);
                dialog.accept(); // Close the dialog when the "Select" button is clicked
            }
        });
        dialog.exec();
        return selectedIndex;
    }


};

#endif //OUTDOORNAV_QT_H


