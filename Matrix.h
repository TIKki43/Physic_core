#pragma once
#include <cstddef>    // std::size_t
#include <algorithm>  // std::copy, std::swap
#include <initializer_list>


template<typename T>
class Matrix{
    T* Data{};
    std::size_t Size{};
    std::size_t* Shape{};
    std::size_t Rank{};

public:
    Matrix() = default;
    // Matrix(const T* inp, std::size_t size) : Data(new T[size]), Size(size) { std::copy(inp, inp + size, Data); }
    Matrix(const T* inp, std::initializer_list<std::size_t> shape) { 
        Rank = shape.size();
        Shape = new std::size_t[Rank];
        std::copy(shape.begin(), shape.end(), Shape); 

        std::size_t realSize{1};
        for (auto i: shape){
            realSize *= i;
        }
        Data = new T[realSize];
        Size = realSize;
        
        std::copy(inp, inp + Size, Data); 
    }

    Matrix(const Matrix& inp){ 
        Rank = inp.Rank;
        Size = inp.Size;
        Shape = new std::size_t[Rank];
        Data = new T[Size];
        std::copy(inp.Data, inp.Data + inp.Size, Data); 
        std::copy(inp.Shape, inp.Shape + Rank, Shape);
    }

    Matrix& operator=(Matrix const & inp){
        T* nData = new T[inp.Size];
        std::size_t* nShape = new std::size_t[inp.Rank];
        std::copy(inp.Data, inp.Data + inp.Size, nData);
        std::swap(Data, nData);
        std::copy(inp.Shape, inp.Shape + inp.Rank, nShape);
        std::swap(Shape, nShape);
        Size = inp.Size;
        Rank = inp.Rank;
        delete[] nData;
        delete[] nShape;
        return *this;
    }
    
    Matrix(Matrix&& inp) noexcept : Data(inp.Data), Size(inp.Size), Shape(inp.Shape), Rank(inp.Rank) {
         inp.Data = nullptr; inp.Shape = nullptr; inp.Size = 0; inp.Rank = 0;  
    }

    Matrix& operator=(Matrix&& inp) noexcept {
        if (this != &inp) {
            delete[] Data;
            delete[] Shape;
            Data = inp.Data;
            Shape = inp.Shape;
            Size = inp.Size;
            Rank = inp.Rank;
            inp.Data = nullptr;
            inp.Shape = nullptr;
            inp.Rank = 0;
            inp.Size = 0;
            
        }
        return *this;
    }

    ~Matrix() { delete[] Data; delete[] Shape; }
}; 
// 1 2 3    2 4 6
// 4 5 7    7 6 0
// 3 5 8 ,  5 2 2 Shapes = (rows, cols, batch, global_pull)
// 1 2 3 4 5 7 3 5 8 ! 2 4 6 7 6 0 5 2 2 
