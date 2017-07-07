#include <Arduino.h>

#define buff_sz 300
#define page_sz 256

class Buffer
{
  public:
    void reset();
    void CarregarBuffer(byte data[],uint8_t num);
    void DescarregarBuffer(byte data[],uint8_t num);
    bool Check();
    bool spaceAvailable(uint16_t numeroBytes);

  private:
    byte posicio[buff_sz];
    uint16_t head;    //Primera posicio lliure
    uint16_t tail;    //Ultima posicio ocupada
    uint16_t posicionsPlenes;
};
