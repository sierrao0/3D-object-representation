#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>

// -------------------------------------------------------------------------
struct Point
{
  float X, Y, Z;
  float distanceTo( const Point& b ) const
    {
      float x = X - b.X;
      float y = Y - b.Y;
      float z = Z - b.Z;
      return( std::sqrt( ( x * x ) + ( y * y ) + ( z * z ) ) );
    }
};

// -------------------------------------------------------------------------

// TODO 1: typedef Graph< Point, float > TGraph | definir el tipo de dato del grafo a utilizar, que soporte un 
// tipo de dato Point como vértice y un valor real como costo de la conexión.

// -------------------------------------------------------------------------
int main( int argc, char* argv[] )
{
  if( argc < 4 )
  {
    std::cerr
      << "Usage: " << argv[ 0 ] << " input_mesh start end"
      << std::endl;
    return( 1 );

  } // fi
  long start_id = std::atoi( argv[ 2 ] );
  long end_id = std::atoi( argv[ 3 ] );

  // TODO 2: TGraph g; | declarar el grafo.

  // Load file in a buffer
  std::ifstream in_mesh_stream( argv[ 1 ], std::ifstream::binary );
  if( !in_mesh_stream )
  {
    std::cerr << "Error reading \"" << argv[ 1 ] << "\"" << std::endl;
    return( 1 );

  } // fi
  in_mesh_stream.seekg( 0, in_mesh_stream.end );
  unsigned long in_mesh_file_length = in_mesh_stream.tellg( );
  in_mesh_stream.seekg( 0, in_mesh_stream.beg );
  char* in_mesh_file_buffer = new char[ in_mesh_file_length ];
  in_mesh_stream.read( in_mesh_file_buffer, in_mesh_file_length );
  in_mesh_stream.close( );
  std::istringstream in_mesh( in_mesh_file_buffer );

  // Read vertices
  long nPoints;
  in_mesh >> nPoints;
  for( long pId = 0; pId < nPoints; pId++ )
  {
    Point pnt;
    in_mesh >> pnt.X >> pnt.Y >> pnt.Z;

    // TODO 3: g.AddVertex( pnt ); | agregar cada punto de la malla como vértice del grafo.

  } // rof

  // Read edges
  long nEdges;
  in_mesh >> nEdges;
  for( long eId = 0; eId < nEdges; eId++ )
  {
    long start, end;
    in_mesh >> start >> end;

    /*
      TODO 4: calcular el costo de la conexión (usando la distancia euclidiana entre puntos) y agregar la arista
      correspondiente en el grafo no dirigido (ambas direcciones de la conexión).
      cost = g.GetVertex( start ).distanceTo( g.GetVertex( end ) );
      g.AddEdge( start, end, cost );
      g.AddEdge( end, start, cost );
    */

  } // rof
  delete [] in_mesh_file_buffer;

  if(
    start_id < 0 || start_id >= vertices.size( ) ||
    end_id < 0 || end_id >= vertices.size( )
    )
  {
    std::cerr << "Invalid path endpoints." << std::endl;
    return( 1 );

  } // fi

  /*
    TODO 5:utilizar un algoritmo de búsqueda de rutas de costo mínimo (como Dijkstra o Prim, por ejemplo),
    para identificar los índices de los vértices que hacen parte de la ruta de costo mínimo, imprimiendo en pantalla
    la cantidad de vértices y las coordenadas de cada uno, en el orden que permita reconstruir la ruta.
    std::vector< long > path = g.Dijkstra( start_id, end_id );
    std::cout << path.size( ) << std::endl;
    for( unsigned int i = 0; i < path.size( ); ++i )
    std::cout
    << vertices[ path[ i ] ].X << " "
    << vertices[ path[ i ] ].Y << " "
    << vertices[ path[ i ] ].Z << std::endl;
  */

  return( 0 );
}

// eof -
