
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <time.h>
#include <map>
#include <utility>

extern "C" {
#include "mcbsp.h"
};

int givenSeed = 0;
int problemSize = 0;

class DLeastCommonSubSequence {
    public:
        typedef std::pair<int, int> CoordsPair;
        typedef std::map<CoordsPair, int**> ChunkMap;
        typedef ChunkMap::const_iterator ChunkMapCIter;
        typedef ChunkMap::iterator ChunkMapIter;

        DLeastCommonSubSequence(int l);
        ~DLeastCommonSubSequence();
        void process();
        CoordsPair getCPair(int i, int j) {
            return std::make_pair<int,int>(i,j);
        }

    private:
        void distributedInit();
        void debugL();

        int n_;
        int id_;

        int s_, t_; // P(s, t) 2D processor naming s, t in [0, n-1]

        char* a_; // for distributed implementation
        char* b_; // these variables holds only repective pieces of whole string
        int chunkPerProc_;
        int chunkLength_; // length of abovementioned pieces
        int chunkStride_; /*     _ _  _
                                |_|_|  | <- this is
                                |_|_| _|    stride measured in chunks 2
                          */

        ChunkMap L_; //
        int length_; // but  length == full problem size
        static const char* alphabet_;
    public:
        static const int procAmount_;
        static const int G;
};

const int
DLeastCommonSubSequence::G = 4;
const char*
DLeastCommonSubSequence::alphabet_ =
        "0123456789"
        "abcdefghijklmnopqrstuvwxyz";
const int
DLeastCommonSubSequence::procAmount_ = 4;


DLeastCommonSubSequence::DLeastCommonSubSequence(int l)
        : length_(l) {
            distributedInit();
}



DLeastCommonSubSequence::~DLeastCommonSubSequence() {
    if(a_)
        delete[] a_;
    if(b_)
        delete[] b_;
    for(ChunkMapIter it = L_.begin(); 
        it != L_.end(); ++it) {
        for(int i = 0; i < G; ++i)
            delete[] it->second[i];
        delete[] it->second;
    }
}


void
DLeastCommonSubSequence::distributedInit() {
    id_ = bsp_pid();
    n_ = bsp_nprocs();
    chunkLength_ = G;
    
    s_ = 1 == ((id_ >> 1) & 0b1);
    t_ = 1 == (id_ & 0b1);
    
    chunkStride_ = length_ / G;
    chunkPerProc_ = chunkStride_ / n_;
    a_ = new char[length_ / n_];
    b_ = new char[length_ / n_];

    for(int i = 0; i < chunkPerProc_ * G; ++i) {
        a_[i] = alphabet_[rand() % strlen(alphabet_)];
        b_[i] = alphabet_[rand() % strlen(alphabet_)];
    }

    for(int i = 0; i < chunkStride_; ++i) {
        for(int j = 0; j < chunkStride_; ++j) {
            if(s_ != i % n_ ||
               t_ != j % n_)
                continue;
            L_[getCPair(i,j)] = new int*[G];
            for(int k = 0; k < G; ++k) {
                L_[getCPair(i,j)][k] = new int [G];
                memset(L_[getCPair(i,j)][k], 0, G*sizeof(int));
            }
        }
    }

    //debug section
    bsp_sync();
    if(0 == id_)
        std::cout << "A" << std::endl;
    bsp_sync();
    for(int i = 0; i < length_; ++i) {
        if(id_ == (i/G) % n_) {
            //std::cout << " i=["<<i<<"],locI=["<< ((i/G)/n_)*G + i%G << "] ";
            std::cout << a_[((i/G)/n_)*G + i%G];
            //             ^__________________^ get local index from global
        }
        bsp_sync();
    }

    if(0 == id_)
        std::cout << std::endl << "B" << std::endl;
    bsp_sync();
    for(int i = 0; i < length_; ++i) {
        if(id_ == (i/G) % n_)
            std::cout << b_[((i/G)/n_)*G + i%G];
        bsp_sync();
    }

    if(0 == id_)
        std::cout << std::endl;
    bsp_sync();
}

void
DLeastCommonSubSequence::debugL() {
    //debug section
    //bsp_sync();
    //for(int i = 0; i < chunkStride_; ++i) {
    //    for(int j = 0; j < chunkStride_; ++j) {
    //        if( s_ == i % n_ &&
    //            t_ == j % n_ ) {
    //            std::cout << "Proc " << id_ << "(" << s_ << ", " <<  t_ 
    //                      << ") holds " << i << " " << j << std::endl;
    //        }
    //        bsp_sync();
    //    }
    //}
    //bsp_abort("stop for a while ");
    //bsp_sync();
    //std::cout << std::endl;
    //for(int i = 0; i < chunkStride_; ++i) {
    //    for(int j = 0; j < chunkStride_; ++j) {
    //        if( s_ == i % n_ &&
    //            t_ == j % n_ ) {
    //            std::cout << "Chunk " << i << ", " << j << " in Proc " 
    //                      << s_ << ", " << t_ << std::endl;
    //            int** ptr = L_[getCPair(i,j)];
    //            for(int bi = 0; bi < chunkLength_; ++bi) {
    //                for(int bj = 0; bj < chunkLength_; ++bj) {
    //                    std::cout << ptr[bi][bj] << " ";
    //                }
    //                std::cout << std::endl;
    //            }
    //            std::cout << "------" << std::endl;
    //        }
    //        bsp_sync();
    //    }
    //}
    //bsp_sync();
}

void
DLeastCommonSubSequence::process() {
    double tStart = bsp_time();
    {
        //for(int i = 1; i <= length_; ++i) {
        //    for(int j = 1; j <= length_; ++j) {
        //        if(a_[i-1] == b_[j-1]) {
        //            L_[i][j] = L_[i-1][j-1] + 1;             
        //            //std::cout << "("<< i << ", " << j << ") Symbol "
        //            //          << a_[i-1] << std::endl;
        //        } else {
        //            L_[i][j] = std::max<int>(L_[i-1][j],
        //                                     L_[i][j-1]);
        //        }
        //    }
        //}
        debugL();       
    }
    double tEnd = bsp_time();
    if(0 == id_)
        std::cout << "Problem size [" << length_ << "]" << std::endl
                  << "Time = " << tEnd - tStart << std::endl;
}


void run() {
    if(givenSeed) {
        srand(givenSeed);
    } else {
        givenSeed = time(NULL);
        std::cout << "Seed: " << givenSeed << std::endl;
    }
    srand(givenSeed);

    bsp_begin(DLeastCommonSubSequence::procAmount_);
    if(problemSize)
        DLeastCommonSubSequence(problemSize).process();
    else
        std::cout << "unreachable" << std::endl;
    bsp_end();
}


int main(int argc, char** argv) {
    bsp_init(run, argc, argv);
    if(argc >= 2) {
        problemSize = atoi(argv[1]);
        if(0 != problemSize % DLeastCommonSubSequence::G) {
            std::cout << "problemSize must be divisible by G" << std::endl;
            return -1;
        }
    } else {
        std::cout << "wrong input" << std::endl;
        return -2;
    }

    if (argc == 3)
        givenSeed = atoi(argv[2]);

    run();
    return 0;
}
