#pragma once
#include <cstddef>    // std::size_t
#include <algorithm>  // std::copy, std::swap
#include <initializer_list>
#include <stdexcept>
#include "Utils.h"
#include <concepts>

template<typename T>
class Matrix{
    T* Data{};
    std::size_t Size{};
    std::size_t* Shape{};
    std::size_t Rank{};

public:
    Matrix() = default;
    // Matrix(const T* inp, std::size_t size) : Data(new T[size]), Size(size) { std::copy(inp, inp + size, Data); }
    Matrix(const T* inp, std::initializer_list<std::size_t> shape) { // inp = intializer list ToDo
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

    Matrix(const T* inp, const std::size_t* shape, std::size_t rank) {
        Rank = rank;
        Shape = new std::size_t[rank];
        std::copy(shape, shape + rank, Shape); 

        std::size_t realSize{1};
        for (std::size_t idx{}; idx < rank; ++idx){
            realSize *= Shape[idx];
        }
        Data = new T[realSize];
        Size = realSize;
        
        std::copy(inp, inp + Size, Data); 
    }

    template<std::size_t N>
    Matrix(const T (&inp)[N]) { 
        Size = N;
        Rank = 1;

        Shape = new std::size_t[Rank];
        Shape[0] = N;

        Data = new T[N];
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
        if (sizeof...(Indices) != Rank) { Utils::panic("Incorrect number of indices"); }
        std::size_t indices[]{static_cast<std::size_t>(idxs)...};
        std::size_t Step{};
        for (std::size_t idx{}; idx < Rank; ++idx){
            if (indices[idx] >= Shape[idx]) { Utils::panic("Matrix index out of range"); }
            Step = Step * Shape[idx] + indices[idx]; // multi rows * cols + cols
        }
        return Data[Step];
    } 

    template<typename... Indices>
    // requires(sizeof...(Indices) == Rank(ToDo expr))
    const T& operator[](Indices... idxs) const {
        if (sizeof...(Indices) != Rank) {Utils::panic("Incorrect number of indices"); }
        std::size_t indices[]{static_cast<std::size_t>(idxs)...};
        std::size_t Step{};
        for (std::size_t idx{}; idx < Rank; ++idx){
            if (indices[idx] >= Shape[idx]) { Utils::panic("Matrix index out of range"); }
            Step = Step * Shape[idx] + indices[idx]; // multi rows * cols + cols
        }
        return Data[Step];
    }
    
    [[nodiscard]] const std::size_t* GetShape() const noexcept {return Shape; }
    [[nodiscard]] const std::size_t GetRank() const noexcept { return Rank; }
    [[nodiscard]] const std::size_t GetSize() const noexcept { return Size; }


    void Reshape(const std::initializer_list<std::size_t> newShape) {
        std::size_t newSize{1};
        for (std::size_t i: newShape) {
            newSize *= i;
        } 
        if (newSize != Size) return Utils::panic("Matrix sizes do not mach");
        std::size_t* nShape = new std::size_t[newShape.size()];
        std::copy(newShape.begin(), newShape.end(), nShape);
        std::swap(nShape, Shape);
        Rank = newShape.size();

        delete[] nShape;
    }

    [[nodiscard]] Matrix operator+(const Matrix& other) const {
        // if ((Rank != other.Rank) || (Size != other.Size)) return;
        if (Rank != other.Rank) {
            Utils::panic("Matrix ranks do not match");
        }

        for (std::size_t i{}; i < Rank; ++i) {
            if (Shape[i] != other.Shape[i]) {
                Utils::panic("Matrix shapes do not match");
            }
        }
        
        T* a = new T[Size];
        for (std::size_t idx{0}; idx < Size; ++idx){
            a[idx] = other.Data[idx] + Data[idx];
        }
        Matrix result(a, Shape, Rank);
        delete[] a;

        return result;
    }

    [[nodiscard]] Matrix operator-(const Matrix& other) const {
        // if ((Rank != other.Rank) || (Size != other.Size)) return;
        if (Rank != other.Rank) {
            Utils::panic("Matrix ranks do not match");
        }

        for (std::size_t i{}; i < Rank; ++i) {
            if (Shape[i] != other.Shape[i]) {
                Utils::panic("Matrix shapes do not match");
            }
        }
        
        T* a = new T[Size];
        for (std::size_t idx{0}; idx < Size; ++idx){
            a[idx] = Data[idx] - other.Data[idx];
        }
        Matrix result(a, Shape, Rank);
        delete[] a;

        return result;
    }
    
    template<typename S>
    requires requires(T values, const S& scalar){ values *= scalar; }
    [[nodiscard]] Matrix operator*(const S& scalar) const{
        Matrix result(*this);
        result *= scalar;
        return result;
    }

    template<typename S>
    requires requires(T values, const S& scalar){ values *= scalar; }
    Matrix& operator*=(const S& scalar) {
        for (std::size_t idx{0}; idx < Size; ++idx){
            Data[idx] *= scalar;
        }
        return *this;
    }

    template<typename S>
    requires requires(T values, const S& scalar){ values += scalar; }
    Matrix& operator+=(const S& scalar) {
        for (std::size_t idx{0}; idx < Size; ++idx){
            Data[idx] += scalar;
        }
        return *this;
    }

    template<typename S>
    requires requires(T values, const S& scalar){ values -= scalar; }
    Matrix& operator-=(const S& scalar) {
        for (std::size_t idx{0}; idx < Size; ++idx){
            Data[idx] -= scalar;
        }
        return *this;
    }
    
    template<typename S>
    requires requires(T values, const S& scalar){ values += scalar; }
    Matrix operator+(const S& scalar) const {
        Matrix result(*this);
        result += scalar;

        return result;
    }

    template<typename S>
    requires requires(T values, const S& scalar){ values -= scalar; }
    Matrix operator-(const S& scalar) const {
        Matrix result(*this);
        result -= scalar;

        return result;
    }

    ~Matrix() { delete[] Data; delete[] Shape; }
}; 
// 1 2 3   2 4 6
// 4 5 7   7 6 0
// 3 5 8   8 0 4
// 5 1 9,  5 2 2 Shapes = (rows, cols, batch, t) [2, 1] [1, 1, 1](Shapes = (4, 3, 2))  [0]
// Size = rows * cols * batch * ... * N * sizeof(T); el = A[rows - 2, cols - 1, batch - 0]
// 1 2 3 4 5 7 3 5 8 5 1 9 !!!!! 2 4 6 7 6 0 8 0 4 5 2 2 
