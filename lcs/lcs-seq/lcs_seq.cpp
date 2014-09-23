
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <time.h>

extern "C" {
#include "mcbsp.h"
};

int givenSeed = 0;
int problemSize = 0;

class LeastCommonSubSequence {
    public:
        LeastCommonSubSequence(int l);
        LeastCommonSubSequence(const char* a,
                               const char* b);

        ~LeastCommonSubSequence();

        void process();
    private:
        char* a_;
        char* b_;
        int** L_;
        int length_;
        static const char* alphabet_;
};


const char*
LeastCommonSubSequence::alphabet_ =
        "0123456789"
        "abcdefghijklmnopqrstuvwxyz";


LeastCommonSubSequence::LeastCommonSubSequence(int l)
        : a_(new char[l+1])
        , b_(new char[l+1])
        , L_(new int*[l+1])
        , length_(l) {
    memset(a_, 0, length_+1);
    memset(b_, 0, length_+1);
    for(int i = 0; i < length_; ++i) {
        a_[i] = alphabet_[rand() % strlen(alphabet_)];
        b_[i] = alphabet_[rand() % strlen(alphabet_)];
    }
    for(int i = 0; i <= length_; ++i) {
        L_[i] = new int[length_+1];
        memset(L_[i], 0, (length_+1) * sizeof(int));
    }
    //std::cout << "A [" << a_ << "]" << std::endl
    //          << "B [" << b_ << "]" << std::endl;
}

LeastCommonSubSequence::LeastCommonSubSequence(const char* a,
                                               const char* b)
        : length_(strlen(a))
        , a_(new char[length_+1])
        , b_(new char[length_+1])
        , L_(new int*[length_+1]) {
    memset(a_, 0, length_+1);
    memset(b_, 0, length_+1);
    memcpy(a_, a, length_);
    memcpy(b_, b, length_);
    for(int i = 0; i <= length_; ++i) {
        L_[i] = new int[length_+1];
        memset(L_[i], 0, (length_+1) * sizeof(int));
    }
    std::cout << "A [" << a_ << "]" << std::endl
              << "B [" << b_ << "]" << std::endl;
}


LeastCommonSubSequence::~LeastCommonSubSequence() {
    if(a_)
        delete[] a_;
    if(b_)
        delete[] b_;
    if(L_) {
        for(int i = 0; i < length_; ++i)
            delete[] L_[i];
        delete[] L_;
    }
}


void
LeastCommonSubSequence::process() {
    double tStart = bsp_time();
    {
        for(int i = 1; i <= length_; ++i) {
            for(int j = 1; j <= length_; ++j) {
                if(a_[i-1] == b_[j-1]) {
                    L_[i][j] = L_[i-1][j-1] + 1;             
                    //std::cout << "("<< i << ", " << j << ") Symbol "
                    //          << a_[i-1] << std::endl;
                } else {
                    L_[i][j] = std::max<int>(L_[i-1][j],
                                             L_[i][j-1]);
                }
            }
        }
        //std::cout << std::endl;
        //for(int i = 0; i < length_+1; ++i) {
        //   for(int j = 0; j < length_+1; ++j) {
        //        std::cout << L_[i][j] << " ";
        //   }
        //   std::cout << std::endl;
        //}
        //   std::cout << std::endl
        //          << "Length : " << L_[length_][length_] << std::endl;
    }
    double tEnd = bsp_time();
    std::cout << "Problem size [" << length_ << "]" << std::endl
              << "Time = " << tEnd - tStart << std::endl
              << "Result, though is " << L_[length_-1][length_-1]
              << std::endl;
}


void run() {
    if(givenSeed) {
        srand(givenSeed);
    } else {
        givenSeed = time(NULL);
        std::cout << "Seed: " << givenSeed << std::endl;
    }
    srand(givenSeed);
    bsp_begin(1);
    if(problemSize)
        LeastCommonSubSequence(problemSize).process();
    else
        LeastCommonSubSequence("mihmmjgfgmoaemcj",
                                "akngjmjfdnmhiihd").process();
        //LeastCommonSubSequence("design", "define").process();
    bsp_end();
}


int main(int argc, char** argv) {
    bsp_init(run, argc, argv);
    if(argc >= 2) {
        problemSize = atoi(argv[1]);
    }
    if (argc == 3)
        givenSeed = atoi(argv[2]);

    run();
    return 0;
}
