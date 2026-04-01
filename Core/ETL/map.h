#ifndef MAP_H
#define MAP_H
#include <array>
#include <cstring>
namespace pyro
{
template <typename K, typename V> class map_t
{

  public: // Linear map list
    constexpr static size_t _max_size = 10;

    map_t()
    {
        //_keys.fill(nullptr);
        //_values.fill(nullptr);
    }
    ~map_t()
    {
        //_keys.fill(nullptr);
        //_values.fill(nullptr);
    }
    V &operator[](const K &key)
    {
        if (exist(key))
        {
            return _values[find(key)];
        }
        else
        {
            _keys[_size]   = key;
            _values[_size] = V();
            _size++;
            return _values[_size - 1];
        }
    }
    V &operator[](const K &key) const
    {
        if (exist(key))
        {
            return _values[find(key)];
        }
        else
        {
            _keys[_size]   = key;
            _values[_size] = V();
            // _size++;
            return _values[_size - 1];
        }
    }

    int find(const K &key)
    {
        for (int i = 0; i < _size; i++)
        {
            if (key == _keys[i])
            {
                return i;
            }
        }
        return -1;
    }
    bool exist(const K &key)
    {
        if (find(key) != -1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    int size()
    {
        return _size;
    }

    void clear()
    {
        _size = 0;
    }

    void erase(const K &key)
    {
        int index = find(key);
        if (index != -1)
        {
            for (int i = index + 1; i < _size; i++)
            {
                _keys[i - 1]   = _keys[i];
                _values[i - 1] = _values[i];
            }
        }
        _size--;
    }

  private:
    int _size;
    std::array<K, _max_size> _keys;
    std::array<V, _max_size> _values;
};
}; // namespace pyro


#endif