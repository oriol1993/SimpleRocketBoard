#include <Arduino.h>

#define buff_sz 300
#define page_sz 256

class Buffer
{
  public:
    void reset();
    void CarregarBuffer(byte data[], int num);
    void DescarregarBuffer();
    bool Check();
    bool spaceAvailable(int numeroBytes);

  private:
    byte posicio[buff_sz];
    unsigned int head;    //Primera posicio lliure
    unsigned int tail;    //Ultima posicio ocupada
    int posicionsPlenes;
};
