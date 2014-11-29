
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

class DLargestCommonSubSequence {
    public:
        typedef std::pair<int, int> CoordsPair;
        typedef std::map<CoordsPair, int**> ChunkMap;
        typedef ChunkMap::const_iterator ChunkMapCIter;
        typedef ChunkMap::iterator ChunkMapIter;
        

        typedef std::vector<int> Row;
        typedef std::vector<Row> Rows;

        DLargestCommonSubSequence(int l);
        ~DLargestCommonSubSequence();
        void process();
        CoordsPair getCPair(int i, int j) {
            return std::make_pair(i,j);
        }

    private:
        void distributedInit();
        
        void printString(const char* name, const char * data);

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
        Rows over_;
                          
        int length_; // but  length == full problem size
        static const char* alphabet_;
    public:
        static const int procAmount_;
};

const char*
DLargestCommonSubSequence::alphabet_ =
        "0123456789"
        "abcdefghijklmnopqrstuvwxyz";
const int
DLargestCommonSubSequence::procAmount_ = 4;


DLargestCommonSubSequence::DLargestCommonSubSequence(int l)
        : length_(l) {
    id_ = bsp_pid();
    n_ = bsp_nprocs();

    chunkStride_ = procAmount_*4;
    chunkLength_ = length_ / chunkStride_;

    chunkPerProc_ = chunkStride_ / n_;

    a_ = new char[length_ / n_];
    b_ = new char[length_ / n_];

    for(int i = 0; i < chunkPerProc_ * chunkLength_; ++i) {
        a_[i] = alphabet_[rand() % strlen(alphabet_)];
        b_[i] = alphabet_[rand() % strlen(alphabet_)];
    }
}



DLargestCommonSubSequence::~DLargestCommonSubSequence() {
    if(a_)
        delete[] a_;
    if(b_)
        delete[] b_;
    for(ChunkMapIter it = L_.begin(); 
        it != L_.end(); ++it) {
        for(int i = 0; i < chunkLength_; ++i)
            delete[] it->second[i];
        delete[] it->second;
    }
}


void
DLargestCommonSubSequence::distributedInit() {
    

    // allocate storage for rows before
    int rowsToExportPerProc = (0 == chunkStride_%n_) ? chunkStride_/n_ :
                                                     chunkStride_/n_+1;  
    //std::cout << "rowsToExportPerProc "
    //          << rowsToExportPerProc << std::endl;

    for(int i = 0; i < rowsToExportPerProc; ++i) {
        Row curr;
        curr.resize(chunkLength_);
        over_.push_back(curr);
        bsp_push_reg(over_.back().data(), sizeof(int)*chunkLength_);
        //std::cout << "Proc " << id_ << 
        //                    " Last Row index " << i <<  " addr: ";
        //                std::cout << over_.back().data()
        //                    << std::endl << std::flush;
    }

    for(int i = 0; i < chunkStride_; ++i) {
        for(int j = 0; j < chunkStride_; ++j) {
            if(id_ != i % n_)
                continue;
            // allocate L 
            L_[getCPair(i,j)] = new int*[chunkLength_];
            for(int k = 0; k < chunkLength_; ++k) {
                L_[getCPair(i,j)][k] = new int [chunkLength_];
                memset(L_[getCPair(i,j)][k], 0, chunkLength_*sizeof(int));
            }

            //allocate chunks for lastRow
            if(i > 0) {
                L_[getCPair(i-1,j)] = new int*[chunkLength_];
                for(int k = 0; k < chunkLength_; ++k) {
                    if(k < chunkLength_-1) {
                        L_[getCPair(i-1,j)][k] = NULL;
                    } else {
                        L_[getCPair(i-1,j)][k] = new int[chunkLength_];
                        memset(L_[getCPair(i-1,j)][k],
                               0, chunkLength_*sizeof(int));
                    }
                }
            }
        }
    }
}


void
DLargestCommonSubSequence::printString(const char* name,
                                     const char * data) {
    bsp_sync();
    if(0 == id_)
        std::cout << name << std::endl;
    bsp_sync();
    for(int i = 0; i < length_; ++i) {
        if(id_ == (i / chunkLength_) % n_) {
            std::cout << data[getLocal(i)];
        }
        bsp_sync();
    }

    if(0 == id_)
        std::cout << std::endl;
    bsp_sync();

}


int
DLargestCommonSubSequence::getLocal(int i) {
    return i % chunkLength_;
}


int
DLargestCommonSubSequence::getLElem(int gi,
                                  int gj) {
    int** chunk = L_[getCPair(gi / chunkLength_, gj / chunkLength_)];
    assert(NULL != chunk);
    assert(NULL != chunk[getLocal(gi)]);
    return chunk[getLocal(gi)][getLocal(gj)];
}


void
DLargestCommonSubSequence::setLElem(int gi,
                                  int gj,
                                  int value) {
    int** chunk = L_[getCPair(gi / chunkLength_, gj / chunkLength_)];
    //std::cout << "gi="<< gi << " gj="<<gj << " And chunk is ["
    //          << gi / chunkLength_ << ", "
    //          << gj / chunkLength_  << "] with local coords {"
    //          << getLocal(gi) << ", "
    //          << getLocal(gj) << "} And val=" << value <<std::endl <<std::flush;
    assert(NULL != chunk);
    assert(NULL != chunk[getLocal(gi)]);
    chunk[getLocal(gi)][getLocal(gj)] = value;
}

void
DLargestCommonSubSequence::calculateChunk(int i,
                                        int j) {
    for(int k = i*chunkLength_; k < (i+1)*chunkLength_; ++k) {
        for(int l = j*chunkLength_; l < (j+1)*chunkLength_; ++l) {
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
DLargestCommonSubSequence::process() {

    printString("A", a_);
    printString("B", b_);

    for(int i = 0; i < n_; ++i) {
        if(id_ == i) {
            std::cout << "Proc {" << i << "}\n" << std::flush;
            distributedInit();
        }
        bsp_sync();
    }
    double tStart = bsp_time();
    {
        for(int a = 0; a < 2*chunkStride_-1; ++a) {
            //if(0 == id_)
            //    std::cout << "Wavefront " << a << std::endl << std::flush;
            
            // copy required elements from over_ buffer to L
            int indexInOver = 0;
            for(int i = 1; i < chunkStride_; ++i) {
                for(int j = 0; j < chunkStride_; ++j) {
                    if( a == i + j &&
                        id_ == i % n_) {
                        memcpy(L_[getCPair(i-1, j)][chunkLength_-1],
                               over_[indexInOver].data(),
                               sizeof(int)*chunkLength_);
                        ++indexInOver;
                    }
                }
            }
            
            // calculate
            for(int i = 0; i < chunkStride_; ++i) {
                for(int j = 0; j < chunkStride_; ++j) {
                    if( a == i + j &&
                        id_ == i % n_) {
                        //std::cout << "calculateChunk[" << i << ", " 
                        //          << j << "] held by proc " << id_
                        //          << std::endl << std::flush;
                        calculateChunk(i, j);
                    }
                }
            }

            // communicate rows 
            indexInOver = 0;
            for(int i = 0; i < chunkStride_; ++i) {
                for(int j = 0; j < chunkStride_; ++j) {
                    if( a == i + j &&
                        id_ == i % n_ &&
                        chunkStride_-1 != i ) {
                        //std::cout << "dstPROC {" << (i+1)%n_ << "} with index ["
                        //          << indexInOver << "], source - from PROC {"
                        //          << id_ << "} with cell ["
                        //          << i <<","<< j 
                        //          << "]" << std::endl << std::flush;
                        //std::cout << "src ptr " << L_[getCPair(i, j)][G-1]
                        //          << std::endl << std::flush;
                        //std::cout << "dst ptr " << over_[indexInOver].data()
                        //          << std::endl << std::flush;
                        bsp_put((i+1)%n_,
                                L_[getCPair(i, j)][chunkLength_-1],
                                over_[indexInOver].data(),
                                0, chunkLength_*sizeof(int));
                        ++indexInOver;
                    }
                }
            }

            bsp_sync();
        }
    }
    double tEnd = bsp_time();
    if(0 == id_)
        std::cout << "Problem size [" << length_ << "]" << std::endl
                  << "Time = " << tEnd - tStart << std::endl;
    bsp_sync();
    if(n_-1 == id_) {
        std::cout << "No way we got a result " <<
            getLElem(length_-1, length_-1) << std::endl;
    }
}


void run() {
    if(givenSeed) {
        srand(givenSeed);
    } else {
        givenSeed = 34; //time(NULL);
        std::cout << "Seed: " << givenSeed << std::endl;
    }
    srand(givenSeed);

    bsp_begin(DLargestCommonSubSequence::procAmount_);
    if(problemSize)
        DLargestCommonSubSequence(problemSize).process();
    else
        std::cout << "unreachable" << std::endl;
    bsp_end();
}


int main(int argc, char** argv) {
    bsp_init(run, argc, argv);
    if(argc >= 2) {
        problemSize = atoi(argv[1]);
        if(0 != problemSize % DLargestCommonSubSequence::procAmount_) {
            std::cout << "problemSize must be divisible by procAmount_"
                      << std::endl;
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
