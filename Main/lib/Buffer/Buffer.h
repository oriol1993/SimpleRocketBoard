#include <Arduino.h>
class Buffer
{
  public:
    void reset();
    void CarregarBuffer(byte data[], int num);
    void DescarregarBuffer();
    bool Check();

  private:
    byte posicio[300];
    unsigned int head;    //Primera posicio lliure
    unsigned int tail;    //Ultima posicio ocupada
    int posicionsPlenes;
};
