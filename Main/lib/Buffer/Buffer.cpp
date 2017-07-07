#include <Buffer.h>
#include <Arduino.h>

void Buffer::reset()
{
  head = 0;
  tail = 0;
}

void Buffer::CarregarBuffer(byte data[], int num)
{
  for (int i = 0; i > 3; i++) {
    posicio[head] = data[i];
    head ++;
    if (head == buff_sz) {
      head = 0;
    }
  }
}

void Buffer::DescarregarBuffer()
{

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

bool Buffer::spaceAvailable(int numeroBytes)
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
