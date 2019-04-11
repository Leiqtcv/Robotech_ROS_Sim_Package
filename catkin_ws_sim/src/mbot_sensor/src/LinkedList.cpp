/**********************************************************
 * 链表的使用库函数
 * 实现链表的数据结构的快速操作
 * *******************************************************/
#include <iostream>
#include "LinkedList.hpp"

using namespace std;

//构造函数
ListCode::ListCode()
{
    //do nothing.
}

//析构函数
ListCode::~ListCode()
{
    //清空链表内存
    clear();
}

//创建链表头节点
void ListCode::create_List()
{
    //头节点没有数据项
    head = new Node(NULL);
}

//从头节点插入一个新节点
void ListCode::insert(const float &add_data)
{
    Node *pTemp = new Node(add_data);
    pTemp->next = head->next;
    head->next = pTemp;
}

//判断链表是否为空
bool ListCode::isEmpty()
{
    Node *pFirst = head;
    if(pFirst->next == NULL)
    {
        return true;
    }
    else
    {
        return false;
    }
}

//判断链表中是否存在某个数据项
bool ListCode::dataisExist(const float &search_data)
{
    Node *pFirst = head, *pEnd = NULL;
    while(pFirst->next != pEnd)
    {
        if(pFirst->next->data == search_data)
        {
            return true;
        }
        pFirst = pFirst->next;
    }
    return false;
}

//在中间某个节点插入数据(有问题)
void ListCode::insert_pos(const float &insert_data, const float &add_data)
{
    Node *p = head, *q = head;
    Node *n = new Node(add_data);
    if(isEmpty() == false)
    {
        while(p != NULL && p->data !=insert_data)
        {
            q = p;
            p = p->next;
        }
        q->next = n;
        n->next = p;
    }
    else
    {
        cout<<"链表为空！"<<endl;
    }
}

//打印链表数据
void ListCode::print()
{
    if(isEmpty()==false)
    {
        cout<<"输出链表:\n"<<endl;
        cout<<"Head"<<"-";
        for(Node *pFirst=head->next; pFirst!=NULL; pFirst=pFirst->next)
        {
            cout<<pFirst->data<<"-";
        }
        cout<<"End"<<endl<<endl;
    }
    else
    {
        cout<<"链表为空！"<<endl;
    }
}

//删除链表中的某一个节点
void ListCode::delete_List(const float &delete_data)
{
    Node *pFirst = head, *pEnd = NULL;
    if(isEmpty()==false)
    {
        while(pFirst->next != pEnd && pFirst->next->data != delete_data)
        {
            pFirst = pFirst->next;
        }
        pFirst->next = pFirst->next->next;
    }
    else
    {
        cout<<"链表为空！"<<endl;
    }
}

//修改链表中的指定的数据
void ListCode::update(const float &old_data, const float &new_data)
{
    Node *pFirst = head, *pEnd = NULL;
    if(isEmpty() == false)
    {
        while(pFirst->next != pEnd && pFirst->next->data != old_data)
        {
            pFirst = pFirst->next;
        }
        if(pFirst->next != pEnd)
        {
            pFirst->next->data = new_data;
            cout<<"将旧数据:"<<old_data<<"替换为新数据:"<<new_data<<endl;
        }
        else
        {
            //cout<<"未在链表中找到要替换的数据."<<endl;
        }
    }
    else
    {
        cout<<"链表为空！"<<endl;
    }
}

//链表的冒泡排序算法(从最小到最大排序)
void ListCode::bubbleSort()
{
    Node *pFirst = head, *pEnd = NULL;

    while(pFirst != pEnd)
    {
        while(pFirst->next != pEnd)
        {
            if(pFirst->data >= pFirst->next->data)
            {
                float temp   = pFirst->data;
                pFirst->data = pFirst->next->data;
                pFirst->next->data = temp;
            }
            pFirst = pFirst->next;
        }
        pEnd = pFirst;
        pFirst = head;
    }
}

//将排序后的链表重复冗余数据删除处理
void ListCode::deleteSortedRepeatedData()
{
    Node *pFirst = head, *pEnd = NULL;
    while((pFirst->next != pEnd) && (pFirst->next->next != pEnd))
    {
        if(pFirst->next->data == pFirst->next->next->data)
        {
            pFirst->next = pFirst->next->next;
        }
        else
        {
            pFirst = pFirst->next;
        }
    }
}

//获取链表的数据项长度
void ListCode::listLength(int &length)
{
    length = 0;
    Node *pFirst = head, *pEnd = NULL;
    while(pFirst->next != pEnd)
    {
        pFirst = pFirst->next;
        (length)++;
    }
}

//获取链表某项索引的数据值
void ListCode::listIndex(int &index, float &value)
{
    int seek_index = 0;
    Node *pFirst = head, * pEnd = NULL;
    while(pFirst->next != pEnd)
    {
        if(seek_index == index)
        {
            value = pFirst->next->data;
            return;
        }
        pFirst = pFirst->next;
        seek_index++;
    }
    cout<<"Index cross-boundary error."<<endl;
}



























