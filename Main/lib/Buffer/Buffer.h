#include <Arduino.h>
class Buffer
{
  public:
    void CarregarBuffer();
    void DescarregarBuffer();
    void Check();

  private:
    byte posicio[300];
    unsigned int head;    //Primera posicio lliure
    unsigned int tail;    //Ultima posicio ocupada

};
