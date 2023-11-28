#ifndef ICELL_HPP
#define ICELL_HPP

// Esta es una clase base abstracta que define la interfaz común de los nodos de instrumentación
class ICell
{
public:
  // Este es un método virtual puro que debe ser implementado por las clases derivadas
  virtual void process() = 0;

  // Este es un destructor virtual para permitir el polimorfismo
  virtual ~ICell() {}
};

#endif // ICELL_HPP
