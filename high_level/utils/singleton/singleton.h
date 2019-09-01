/**
 * @author koseng : lintangerlangga@gmail.com
 */

/*
 * Meyer's Singleton
 * I prefer to use reference because unnecessary to manage memory
 */

#pragma once

#include <boost/thread/mutex.hpp>

template <class T>
class Singleton{
protected:
//    Singleton(){}

private:
//    Singleton(const Singleton&){}
//    Singleton& operator=(const Singleton&){}
    static boost::mutex mtx_;
public:
    virtual ~Singleton() {}
    static T& getInstance();
};

template <class T>
boost::mutex Singleton<T>::mtx_;

template <class T>
T& Singleton<T>::getInstance(){
    boost::mutex::scoped_lock lock(mtx_);
    static T instance;
    return instance;
}
