/**********************************************************
 * 链表的使用库函数
 * 实现链表的数据结构的快速操作
 * *******************************************************/
#include <iostream>

using namespace std;

class ListCode
{
public:
    //链表类构造函数和析构函数
    ListCode();
    ~ListCode();

    //链表处理函数库
    bool isEmpty();
    void create_List();
    void insert(const float &add_data);
    void print();
    void delete_List(const float &delete_data);
    void update(const float &old_data, const float &new_data);
    void insert_pos(const float &insert_data, const float &add_data);
    void bubbleSort();
    void deleteSortedRepeatedData();
    void listLength(int &length);
    void listIndex(int &index, float &value);
    bool dataisExist(const float &search_data);

private:
    //链表数据结构
    struct Node
    {
        float data;
        Node *next;
        Node(const float &init_data) : data(init_data), next(NULL){}
    };
    //链表头节点指针
    Node *head;

    //清理链表(在析构函数中调用)
    void clear()
    {
        Node *pFirst = head;
        while(pFirst)
        {
            Node *qTemp = pFirst->next;
            delete pFirst;
            pFirst = qTemp;
        }
    }
};






