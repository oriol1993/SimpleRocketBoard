#include <Arduino.h>
class Buffer
{
  public:
    void CarregarBuffer();
    void DescarregarBuffer();
    void Check();

  private:
    uint8_t posicio[300];
    uint16_t head;    //Primera posicio lliure
    uint16_t tail;    //Ultima posicio ocupada
};
