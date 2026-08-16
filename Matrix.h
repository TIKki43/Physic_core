#pragma once
#include <cstddef>    // std::size_t
#include <algorithm>  // std::copy, std::swap
#include <initializer_list>
#include <stdexcept>

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

    template<typename... Indices>
    // requires(sizeof...(Indices) == Rank(ToDo expr))
    T& operator[](Indices... idxs){
        if (sizeof...(Indices) != Rank) { throw std::invalid_argument("Incorrect number of indices"); }
        std::size_t indices[]{static_cast<std::size_t>(idxs)...};
        std::size_t Step{};
        for (std::size_t idx{}; idx < Rank; ++idx){
            if (indices[idx] >= Shape[idx]) { throw std::out_of_range("Matrix index out of range"); }
            Step = Step * Shape[idx] + indices[idx]; // multi rows * cols + cols
        }
        return Data[Step];
    }

    ~Matrix() { delete[] Data; delete[] Shape; }
}; 
// 1 2 3   2 4 6
// 4 5 7   7 6 0
// 3 5 8   8 0 4
// 5 1 9,  5 2 2 Shapes = (rows, cols, batch, t) [2, 1] [1, 1, 1](Shapes = (4, 3, 2))  [0]
// Size = rows * cols * batch * ... * N * sizeof(T); el = A[rows - 2, cols - 1, batch - 0]
// 1 2 3 4 5 7 3 5 8 5 1 9 !!!!! 2 4 6 7 6 0 8 0 4 5 2 2 
