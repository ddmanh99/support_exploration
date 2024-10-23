#include <iostream>
#include <cmath> // Thêm thư viện toán học

int main()
{
    while (true)
    {
        double a;
        std::cout << "Nhap 1 so: ";
        std::cin >> a;
        std::cout << "So duoc lam tron: ";
        std::cout << round(a) << std::endl;
    }
    return 0; // Thêm câu lệnh return 0
}
