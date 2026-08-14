#include "EstadoRobo.h"


EstadoRoboControl::EstadoRoboControl()
{
    estadoAtual =
        ESTADO_SEGUINDO_LINHA;
}


void EstadoRoboControl::definirEstado(
    EstadoRobo novoEstado
)
{
    estadoAtual =
        novoEstado;
}


EstadoRobo EstadoRoboControl::getEstado()
{
    return estadoAtual;
}