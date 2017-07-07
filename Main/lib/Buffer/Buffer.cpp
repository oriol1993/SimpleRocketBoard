#include <Buffer.h>
#include <Arduino.h>

void Buffer::reset()
{
  head = 0;
  tail = 0;
}

void Buffer::CarregarBuffer(byte data[], uint8_t num)
{
  for (uint8_t i = 0; i < num; i++) {
    posicio[head++] = data[i];
    if (head == buff_sz) {
      head = 0;
    }
  }
}

void Buffer::DescarregarBuffer(byte data[],uint8_t num)
{
  for (uint8_t i = 0; i < num; i++) {
    data[i] = posicio[tail++];
    if (tail == buff_sz) {
      tail = 0;
    }
  }
}

bool Buffer::Check()
{
  if (head > tail) {
    posicionsPlenes = head - tail;
  }
  else {
    posicionsPlenes = buff_sz - (tail - 1) + head;
  }

  if (posicionsPlenes > page_sz) {
    return true;
  }
  else {
    return false;
  }
}

bool Buffer::spaceAvailable(uint16_t numeroBytes)
{
  if (head > tail) {
    if ((buff_sz - (head - tail)) > numeroBytes) {return true;}
    else {return false;}
  }
  else {
    if ((buff_sz - (buff_sz - (tail - 1) + head)) > numeroBytes) {return true;}
    else {return false;}
  }
}
