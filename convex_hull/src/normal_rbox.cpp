
#include <iostream>
#include <string>
#include <random>

int main (int argc, char** argv) {
    if (argc != 4 ||
        std::string (argv[2]) != "D2")
    {   return -1;
    }
    if (std::string (argv[3]).at (0) != 't')
    {   return -2;
    }
    int seed = std::stoi (std::string (argv[3]).substr (1));

    int size = std::stoi (argv [1]);
    std::cout << "2 normal_rbox " << size << " D2 t" << seed << std::endl;
    std::cout << size << std::endl;

    std::mt19937 generator (seed);
    std::normal_distribution<double> distribution (0.0, 0.2);

    std::cout.setf (std::ios_base::fixed, std::ios_base::floatfield);
    std::cout.precision (17);
    while (size > 0)
    {   double x = distribution (generator);
        double y = distribution (generator);
        // if (std::abs (x) < 0.5 && std::abs (y) < 0.5)
        {   std::cout << x << " " << y << std::endl;
            --size;
        }
    }
    return 0;
}
