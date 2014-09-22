
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <time.h>
#include <map>
#include <utility>
#include <cmath>
#include <cassert> 
#include <vector>

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
            return std::make_pair(i,j);
        }

    private:
        void distributedInit();
        void debugL();
        void retrieveLastRowInBlock(int srcI, int srcJ,  // represented here
                                    int dstI, int dstJ); // indeces count chunks
        void calculateChunk(int i, int j);

        int getLocal(int gIndex);  // convert global index [0..length]
                                   // to local [0..chunkLength]

        int getLElem(int i, int j);// given global indexes of L entry
                                   // get actual entry value
        void setLElem(int i, int j, int value);

        int n_;
        int id_;

        // int s_, t_; // P(s, t) 2D processor naming s, t in [0, n-1]

        char* a_; // for distributed implementation
        char* b_; // these variables holds only repective pieces of whole string
        
        int chunkLength_; // length of abovementioned pieces
        int chunkStride_; /*     _ _  _
                                |_|_|  | <- this is
                                |_|_| _|    stride measured in chunks 2
                          */
        //int procStride_;
        int chunkPerProc_;

        ChunkMap L_;
                          
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
    id_ = bsp_pid();
    n_ = bsp_nprocs();
    chunkLength_ = G;
    
    //s_ = 1 == ((id_ >> 1) & 0b1);
    //t_ = 1 == (id_ & 0b1);
    
    chunkStride_ = length_ / G;
    //procStride_ = sqrt(n_);
    chunkPerProc_ = chunkStride_ / n_;
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
    a_ = new char[length_ / n_];
    b_ = new char[length_ / n_];

    for(int i = 0; i < chunkPerProc_ * G; ++i) {
        a_[i] = alphabet_[rand() % strlen(alphabet_)];
        b_[i] = alphabet_[rand() % strlen(alphabet_)];
    }

    for(int i = 0; i < chunkStride_; ++i) {
        for(int j = 0; j < chunkStride_; ++j) {
            if(id_ != i % n_)
                continue;
            // allocate L 
            L_[getCPair(i,j)] = new int*[G];
            for(int k = 0; k < G; ++k) {
                L_[getCPair(i,j)][k] = new int [G];
                memset(L_[getCPair(i,j)][k], 0, G*sizeof(int));
                if(G-1 == k) {
                    bsp_push_reg(L_[getCPair(i,j)][k], G*sizeof(int));
                    std::cout << "Proc " << id_ << 
                            " Last Row in chunk [" << i << ", " <<
                            j << "] addr: ";
                        std::cout << L_[getCPair(i,j)][k]
                            << std::endl << std::flush;
                }
            }

            //allocate chunks for lastRow
            if(i > 0) {
                L_[getCPair(i-1,j)] = new int*[G];
                for(int k = 0; k < G; ++k) {
                    if(k < G-1) {
                        L_[getCPair(i-1,j)][k] = nullptr;
                    } else {
                        L_[getCPair(i-1,j)][k] = new int[G];
                        memset(L_[getCPair(i-1,j)][k], 0, G*sizeof(int));
                        bsp_push_reg(L_[getCPair(i-1,j)][k], G*sizeof(int));
                        std::cout << "Proc " << id_ << 
                                  " Last Row in chunk [" << i-1 << ", " <<
                                  j << "] addr: ";
                        std::cout << L_[getCPair(i-1,j)][k]
                                  << std::endl << std::flush;
                    }
                }
            }
        }
    }

    //debug section
    //bsp_sync();
    //if(0 == id_)
    //    std::cout << "A" << std::endl;
    //bsp_sync();
    //for(int i = 0; i < length_; ++i) {
    //    if(id_ == (i/G) % n_) {
    //        //std::cout << " i=["<<i<<"],locI=["<< ((i/G)/n_)*G + i%G << "] ";
    //        std::cout << a_[((i/G)/n_)*G + i%G];
    //        //             ^__________________^ get local index from global
    //    }
    //    bsp_sync();
    //}

    //if(0 == id_)
    //    std::cout << std::endl << "B" << std::endl;
    //bsp_sync();
    //for(int i = 0; i < length_; ++i) {
    //    if(id_ == (i/G) % n_)
    //        std::cout << b_[((i/G)/n_)*G + i%G];
    //    bsp_sync();
    //}

    //if(0 == id_)
    //    std::cout << std::endl;
    //bsp_sync();
}

void
DLeastCommonSubSequence::debugL() {
    // debug section
    //bsp_sync();
    //for(int i = 0; i < chunkStride_; ++i) {
    //    for(int j = 0; j < chunkStride_; ++j) {
    //        //if(0 == id_)
    //        //    std::cout<< "Chunk {" << i << ", " << j << "}" << std::endl <<
    //        //                "would be handled by proc (" <<
    //        //                i % procStride_ << ", " <<
    //        //                j % procStride_ << ")"<< std::endl;
    //        if( s_ == i % procStride_ &&
    //            t_ == j % procStride_ ) {
    //            std::cout << "Proc " << id_ << "(" << s_ << ", " <<  t_ 
    //                      << ") holds " << i << " " << j << std::endl;
    //        }
    //        bsp_sync();
    //    }
    //}
    //bsp_sync();
    //bsp_abort("stop for a while\n");
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


int
DLeastCommonSubSequence::getLocal(int i) {
    return ((i/G)/n_)*G + i%G;
}


void
DLeastCommonSubSequence::retrieveLastRowInBlock(int srcI, int srcJ,
                                                int dstI, int dstJ) {
    //                     yep, this guy -> v
    // col prev block getting             _|_|
    // row prev block locally colocated  |_|X|
    std::cout << "dstPROC {" << id_ << "} with cell ["<< dstI <<","<< dstJ
              << "], getting from PROC {" << srcI%n_ << "} with cell ["
              << srcI <<","<< srcJ
              << "]" << std::endl << std::flush;
    std::cout << "src ptr " << L_[getCPair(srcI, srcJ)][G-1] << std::endl;
    std::cout << "dst ptr " << L_[getCPair(dstI, dstJ)][G-1] << std::endl;

    bsp_get(srcI % n_, L_[getCPair(srcI, srcJ)][G-1], 0,
            L_[getCPair(dstI,dstJ)][G-1], sizeof(int)*chunkLength_);
    bsp_sync();
}

int
DLeastCommonSubSequence::getLElem(int gi,
                                  int gj) {
    int** chunk = L_[getCPair(gi / chunkStride_, gj / chunkStride_)];
    assert(NULL != chunk);
    return chunk[getLocal(gi)][getLocal(gj)];
}


void
DLeastCommonSubSequence::setLElem(int gi,
                                  int gj,
                                  int value) {
    int** chunk = L_[getCPair(gi / chunkStride_, gj / chunkStride_)];
    //std::cout << "gi="<<gi << " gj="<<gj << " And chunk is ["
    //          << gi / chunkStride_ << ", "
    //          << gj / chunkStride_  << "]"
    //          << " And val=" << value <<std::endl <<std::flush;
    assert(NULL != chunk);
    chunk[getLocal(gi)][getLocal(gj)] = value;
}

void
DLeastCommonSubSequence::calculateChunk(int i,
                                        int j) {
    for(int k = i*G; k < (i+1)*G; ++k) {
        for(int l = j*G; l < (j+1)*G; ++l) {
            if( 0 == k ||
                0 == l ) {
                setLElem(k, l, 0);
            } else {
                char iSymbol = a_[getLocal(k)];
                char jSymbol = b_[getLocal(l)];
                if(iSymbol == jSymbol) {
                    setLElem(k,l, getLElem(k-1, l-1) + 1);
                } else {
                    setLElem(k, l, std::max<int>(getLElem(k-1, l),
                                                 getLElem(k, l-1)));
                }
            }
        }
    }
}


void
DLeastCommonSubSequence::process() {
    for(int i = 0; i < n_; ++i) {
        if(id_ == i)
            distributedInit();
        bsp_sync();
    }
    double tStart = bsp_time();
    {
        for(int a = 0; a < 2*chunkStride_-1; ++a) {
            bsp_sync();
            if(0 == id_)
                std::cout << "Wavefront " << a << std::endl << std::flush;

            bsp_sync();
            // communicate rows 
            for(int i = 0; i < chunkStride_; ++i) {
                for(int j = 0; j < chunkStride_; ++j) {
                    if( a == i + j &&
                        id_ == i % n_ &&
                        0 != i )
                        retrieveLastRowInBlock(i-1, j, i, j);
                }
            }

            bsp_sync();
            // calculate
            for(int i = 0; i < chunkStride_; ++i) {
                for(int j = 0; j < chunkStride_; ++j) {
                    if( a == i + j &&
                        id_ == i % n_) {
                        std::cout << "calculateChunk[" << i << ", " 
                                  << j << "] held by proc " << id_
                                  << std::endl << std::flush;
                        //calculateChunk(i, j);
                    }
                    bsp_sync();
                }
            }
            bsp_sync();
            if(0 == id_) {
                std::cout << "================\n";
            }
            bsp_sync();
        }

        for(ChunkMapIter it = L_.begin();
            it != L_.end(); ++it) {
            if(nullptr == it->second[0])
            bsp_pop_reg(it->second[G-1]);
        }

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
