#pragma once
#include <vector>

template<typename T>
class Buffer
{
private:
    int nMaxSize;
    std::vector<T> dataVector;

public:
    Buffer(int nMaxSize);
    ~Buffer();
    void addData(T newData);
    T* data();
    size_t size();
};

template<typename T>
Buffer<T>::Buffer(int nMaxSize_) : nMaxSize(nMaxSize_)
{
    dataVector.reserve(nMaxSize);
}

template<typename T>
Buffer<T>::~Buffer(){}

template<typename T>
void Buffer<T>::addData(T newData)
{
    if (dataVector.size() >= nMaxSize)
    {
        dataVector.erase(dataVector.begin());
    }
    dataVector.push_back(newData);
}

template<typename T>
T* Buffer<T>::data()
{
    return dataVector.data();
}

template<typename T>
size_t Buffer<T>::size()
{
    return dataVector.size();
}

