#include "LinkedList.hpp"
#include <iostream>

using namespace std;

int main()
{ 
    int length = 0;
    ListCode list;
    list.create_List();
    list.insert(0);
    list.insert(0);
    list.insert(70);
    list.insert(30);
    list.insert(50);
    list.insert(30);
    list.insert(80);
    list.insert_pos(50,30);
    list.print();

    list.bubbleSort();
    list.listLength(length);
    cout<<"length:"<<length<<endl;
    list.print();

    list.update(3,100);
    list.print();

}






