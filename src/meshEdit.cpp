#include "meshEdit.h"
#include "error_dialog.h"
#include "mutablePriorityQueue.h"
#include <assert.h>
#include <float.h>
#include <vector>

namespace CMU462 {

std::vector<HalfedgeIter> findHalfedgesForVertex(HalfedgeIter h) {
  std::vector<HalfedgeIter> h_list;
  auto f = h;
  do {
    h_list.push_back(f);
    f = f->twin()->next();
  } while (f != h);

  return h_list;
}

std::vector<HalfedgeIter> findHalfedgesForFace(HalfedgeIter h) {
  std::vector<HalfedgeIter> h_list{h};
  auto f = h->next();
  while (f != h) {
    h_list.push_back(f);
    f = f->next();
  }
  return h_list;
}

std::vector<FaceIter> findFaceForVertex(Vertex &v) {
  vector<FaceIter> f_list;
  auto h = v.halfedge();

  while (true) {
    f_list.push_back(h->face());
    h = h->twin()->next();
    if (h == v.halfedge()) {
      break;
    }
  }
  return f_list;
}

std::vector<FaceIter> findFaceForVertex(VertexIter v) {
  return findFaceForVertex(*v);
}

VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
  // TODO: (meshEdit)
  // This method should split the given edge and return an iterator to the
  // newly inserted vertex. The halfedge of this vertex should point along
  // the edge that was split, rather than the new edges.

  showError("splitEdge() not implemented.");
  return VertexIter();
}

VertexIter HalfedgeMesh::collapseEdge(EdgeIter e) {
  // TODO: (meshEdit)
  // This method should collapse the given edge and return an iterator to
  // the new vertex created by the collapse.
  auto h00 = e->halfedge();
  auto h01 = h00->twin();
  if(h01->next()->edge()->isBoundary()) {
    h01 = e->halfedge();
    h00 = h01->twin();
  }

  auto f0_hlist = findHalfedgesForFace(h00);
  auto f1_hlist = findHalfedgesForFace(h01);

  auto h001 = f0_hlist[1];
  auto h011 = f1_hlist[1];
  auto h00n = f0_hlist.back();
  auto h01n = f1_hlist.back();

  // Vertices
  auto v0 = h00->vertex();
  auto v1 = h01->vertex();

  // Faces
  auto f0 = h00->face();
  auto f1 = h01->face();
//  if (dot(f0->normal(), f1->normal()) < 0) {
//    return verticesEnd();
//  }


  auto v0_hlist = findHalfedgesForFace(h00);


  // Reassign elements.
  //  v1->position = (v0->position + v1->position) / 2;

  // Related to v0
  auto h0i = h00;
  do {
    h0i = h0i->twin()->next();
    h0i->vertex() = v1;
    //    cout << h0i->getInfo()[2] << endl;
  } while (h0i != h00);

  // Related to h00
  h00n->next() = h001; // h00n->next() = h00
  f0->halfedge() = h001;
  v0->halfedge() = h011;

  // Related to h01
  h01n->next() = h011;
  f1->halfedge() = h01n;
  v1->halfedge() = h001;

  // delete
  deleteHalfedge(h00);
  deleteHalfedge(h01);
  deleteEdge(e);
  deleteVertex(v0);

  if (f0_hlist.size() == 3) {
    auto e1 = h00n->edge();
    auto h00n_twin = h00n->twin();

    //  h00n->next(): Don't worry about it?
    //  h00n->twin(): to be deleted.
    h00n->vertex()->halfedge() = h001->twin();
    //  h00n->edge(): to be deleted.
    //  h00n->face(): to be deleted.
    // h00n->prev:
    h001->next() = h00n->twin()->next();

    //  h00n->twin()->next(): Don't worry about it?
    //  h00n->twin()->twin(): to be deleted.
    //  h00n->twin()->vertex(): v1, v1->halfedge() is h001.
    //  h00n->twin()->edge(): e1, to be deleted.
    h00n->twin()->face()->halfedge() = h00n->twin()->next();
    // h00n->twin()->prev:
    auto v1_hlist = findHalfedgesForVertex(h00n->twin());
    v1_hlist.back()->twin()->next() = h001;

    h001->face() = h00n_twin->face();

    deleteHalfedge(h00n);
    deleteHalfedge(h00n_twin);
    deleteEdge(e1);
    if (f0->isBoundary()) {
      deleteBoundary(f0);
    } else {
      deleteFace(f0);
    }
  }

  if (f1_hlist.size() == 3) {
    auto e2 = h011->edge();
    auto h011_twin = h011->twin();

    //  h011->next(): Don't worry about it?
    //  h011->twin(): to be deleted.
    //  h011->vertex()->halfedge(): v1, v1->halfedge() is h001.
    //  h011->edge(): to be deleted.
    //  h011->face(): to be deleted.
    //  h011->prev:
    h01n->next() = h011_twin->next();

    //  h011_twin->next(): Don't worry about it?
    //  h011_twin->twin(): to be deleted.
    h011_twin->vertex()->halfedge() = h01n;
    //  h011_twin->edge(): e1, to be deleted.
    h011_twin->face()->halfedge() = h01n;
    // h011_twin->prev:
    auto v3_hlist = findHalfedgesForVertex(h011_twin);
    v3_hlist.back()->twin()->next() = h01n;

    h01n->face() = h011_twin->face();

    deleteHalfedge(h011);
    deleteHalfedge(h011_twin);
    deleteEdge(e2);
    if (f1->isBoundary()) {
      deleteBoundary(f1);
    } else {
      deleteFace(f1);
    }
  }

  return v1;
}

VertexIter HalfedgeMesh::collapseFace(FaceIter f) {
  // TODO: (meshEdit)
  // This method should collapse the given face and return an iterator to
  // the new vertex created by the collapse.
  showError("collapseFace() not implemented.");
  return VertexIter();
}

FaceIter HalfedgeMesh::eraseVertex(VertexIter v) {
  // TODO: (meshEdit)
  // This method should replace the given vertex and all its neighboring
  // edges and faces with a single face, returning the new face.

  return FaceIter();
}

FaceIter HalfedgeMesh::eraseEdge(EdgeIter e) {
  // TODO: (meshEdit)
  // This method should erase the given edge and return an iterator to the
  // merged face.

  showError("eraseVertex() not implemented.");
  return FaceIter();
}

EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
  // TODO: (meshEdit)
  // This method should flip the given edge and return an iterator to the
  // flipped edge.
  if (e0->isBoundary()) {
    return e0;
  }
  auto h00 = e0->halfedge();
  auto h01 = h00->twin();
  auto h00_list = findHalfedgesForFace(h00);
  auto h01_list = findHalfedgesForFace(h01);

  if (h00_list.size() < 3 || h01_list.size() < 3) {
    return e0;
  }

  auto h001 = h00_list[1];
  auto h002 = h00_list[2];
  auto h00n = h00_list.back();
  auto h011 = h01_list[1];
  auto h012 = h01_list[2];
  auto h01n = h01_list.back();

  //  auto h6 = h001->twin();
  //  auto h7 = h00n->twin();
  //  auto h8 = h011->twin();
  //  auto h9 = h01n->twin();

  // Vertices
  VertexIter v0 = h00->vertex();
  VertexIter v1 = h01->vertex();
  VertexIter v2 = h012->vertex();
  VertexIter v3 = h002->vertex();

  // Edges
  EdgeIter e1 = h01n->edge();
  EdgeIter e2 = h011->edge();
  EdgeIter e3 = h00n->edge();
  EdgeIter e4 = h001->edge();

  // Faces
  FaceIter f0 = h00->face();
  FaceIter f1 = h01->face();

  // Reassign elements.
  h00->next() = h002;
  h00->twin() = h01;
  h00->vertex() = v2;
  h00->edge() = e0;
  h00->face() = f0;

  h001->next() = h01;
  //  h001->twin() = h7;
  //  h001->vertex() = v3;
  //  h001->edge() = e3;
  h001->face() = f1;

  h00n->next() = h011;
  //  h00n->twin() = h8;
  //  h00n->vertex() = v0;
  //  h00n->edge() = e2;
  //  h00n->face() = f0;

  h01->next() = h012;
  h01->twin() = h00;
  h01->vertex() = v3;
  h01->edge() = e0;
  h01->face() = f1;

  h011->next() = h00;
  //  h011->twin() = h9;
  //  h011->vertex() = v2;
  //  h011->edge() = e1;
  h011->face() = f0;

  h01n->next() = h001;
  //  h01n->twin() = h6;
  //  h01n->vertex() = v1;
  //  h01n->edge() = e4;
  //  h01n->face() = f1;

  //  h6->twin() = h01n;
  //  h7->twin() = h001;
  //  h8->twin() = h00n;
  //  h9->twin() = h011;

  // Vertices
  v0->halfedge() = h011;
  v1->halfedge() = h001;
  v2->halfedge() = h00;
  v3->halfedge() = h01;

  // EDGES
  //  e0->halfedge() = h00;
  //  e1->halfedge() = h01n;
  //  e2->halfedge() = h011;
  //  e3->halfedge() = h00n;
  //  e4->halfedge() = h001;

  // Faces
  f0->halfedge() = h00;
  f1->halfedge() = h01;

  return e0;
}

void HalfedgeMesh::subdivideQuad(bool useCatmullClark) {
  // Unlike the local mesh operations (like bevel or edge flip), we will perform
  // subdivision by splitting *all* faces into quads "simultaneously."  Rather
  // than operating directly on the halfedge data structure (which as you've
  // seen
  // is quite difficult to maintain!) we are going to do something a bit nicer:
  //
  //    1. Create a raw list of vertex positions and faces (rather than a full-
  //       blown halfedge mesh).
  //
  //    2. Build a new halfedge mesh from these lists, replacing the old one.
  //
  // Sometimes rebuilding a data structure from scratch is simpler (and even
  // more
  // efficient) than incrementally modifying the existing one.  These steps are
  // detailed below.

  // TODO Step I: Compute the vertex positions for the subdivided mesh.  Here
  // we're
  // going to do something a little bit strange: since we will have one vertex
  // in
  // the subdivided mesh for each vertex, edge, and face in the original mesh,
  // we
  // can nicely store the new vertex *positions* as attributes on vertices,
  // edges,
  // and faces of the original mesh.  These positions can then be conveniently
  // copied into the new, subdivided mesh.
  // [See subroutines for actual "TODO"s]
  if (useCatmullClark) {
    computeCatmullClarkPositions();
  } else {
    computeLinearSubdivisionPositions();
  }

  // TODO Step II: Assign a unique index (starting at 0) to each vertex, edge,
  // and
  // face in the original mesh.  These indices will be the indices of the
  // vertices
  // in the new (subdivided mesh).  They do not have to be assigned in any
  // particular
  // order, so long as no index is shared by more than one mesh element, and the
  // total number of indices is equal to V+E+F, i.e., the total number of
  // vertices
  // plus edges plus faces in the original mesh.  Basically we just need a
  // one-to-one
  // mapping between original mesh elements and subdivided mesh vertices.
  // [See subroutine for actual "TODO"s]
  assignSubdivisionIndices();

  // TODO Step III: Build a list of quads in the new (subdivided) mesh, as
  // tuples of
  // the element indices defined above.  In other words, each new quad should be
  // of
  // the form (i,j,k,l), where i,j,k and l are four of the indices stored on our
  // original mesh elements.  Note that it is essential to get the orientation
  // right
  // here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces should
  // circulate in the same direction as old faces (think about the right-hand
  // rule).
  // [See subroutines for actual "TODO"s]
  vector<vector<Index>> subDFaces;
  vector<Vector3D> subDVertices;
  buildSubdivisionFaceList(subDFaces);
  buildSubdivisionVertexList(subDVertices);

  // TODO Step IV: Pass the list of vertices and quads to a routine that clears
  // the
  // internal data for this halfedge mesh, and builds new halfedge data from
  // scratch,
  // using the two lists.
  rebuild(subDFaces, subDVertices);
}

/**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * simple linear interpolation, e.g., the edge midpoints and face
 * centroids.
 */
void HalfedgeMesh::computeLinearSubdivisionPositions() {
  // TODO For each vertex, assign Vertex::newPosition to
  // its original position, Vertex::position.
  for (auto &vertex : vertices) {
    vertex.newPosition = vertex.position;
  }

  // TODO For each edge, assign the midpoint of the two original
  // positions to Edge::newPosition.
  for (auto &edge : edges) {
    const auto &p0 = edge.halfedge()->vertex()->position;
    const auto &p1 = edge.halfedge()->twin()->vertex()->position;
    edge.newPosition = (p0 + p1) / 2;
  }

  // TODO For each face, assign the centroid (i.e., arithmetic mean)
  // of the original vertex positions to Face::newPosition.  Note
  // that in general, NOT all faces will be triangles!
  for (auto &face : faces) {
    auto h_list = findHalfedgesForFace(face.halfedge());
    Vector3D p(0, 0, 0);

    for (const auto &h : h_list) {
      p += h->vertex()->position;
    }
    face.newPosition = p / h_list.size();
  }
}

/**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * the Catmull-Clark rules for subdivision.
 */
void HalfedgeMesh::computeCatmullClarkPositions() {
  // TODO The implementation for this routine should be
  // a lot like HalfedgeMesh::computeLinearSubdivisionPositions(),
  // except that the calculation of the positions themsevles is
  // slightly more involved, using the Catmull-Clark subdivision
  // rules. (These rules are outlined in the Developer Manual.)

  // TODO face
  for (auto &face : faces) {
    auto h_list = findHalfedgesForFace(face.halfedge());
    Vector3D p(0, 0, 0);

    for (const auto &h : h_list) {
      p += h->vertex()->position;
    }
    face.newPosition = p / h_list.size();
  }

  // TODO edges
  for (auto &edge : edges) {
    Vector3D pa = edge.halfedge()->vertex()->position;
    Vector3D pb = edge.halfedge()->twin()->vertex()->position;
    Vector3D pc = edge.halfedge()->face()->newPosition;
    Vector3D pd = edge.halfedge()->twin()->face()->newPosition;
    edge.newPosition = (pa + pb + pc + pd) / 4;
  }

  // TODO vertices
  for (auto &vertex : vertices) {
    Vector3D S = vertex.position;
    Vector3D Q, R;

    auto h = vertex.halfedge();
    int n = 0;
    while (true) {
      Q += h->face()->newPosition;
      R += (h->vertex()->position + h->twin()->vertex()->position) / 2;

      n += 1;
      h = h->twin()->next();
      if (h == vertex.halfedge()) {
        break;
      }
    }

    Q /= n;
    R /= n;

    vertex.newPosition = (Q + 2 * R + (n - 3) * S) / n;
  }
}

/**
 * Assign a unique integer index to each vertex, edge, and face in
 * the mesh, starting at 0 and incrementing by 1 for each element.
 * These indices will be used as the vertex indices for a mesh
 * subdivided using Catmull-Clark (or linear) subdivision.
 */
void HalfedgeMesh::assignSubdivisionIndices() {
  // TODO Start a counter at zero; if you like, you can use the
  // "Index" type (defined in halfedgeMesh.h)
  Index idx = 0;

  // TODO Iterate over vertices, assigning values to Vertex::index
  for (auto &vertex : vertices) {
    vertex.index = idx;
    idx++;
  }

  // TODO Iterate over edges, assigning values to Edge::index
  for (auto &edge : edges) {
    edge.index = idx;
    idx++;
  }

  // TODO Iterate over faces, assigning values to Face::index
  for (auto &face : faces) {
    face.index = idx;
    idx++;
  }
}

/**
 * Build a flat list containing all the vertex positions for a
 * Catmull-Clark (or linear) subdivison of this mesh.  The order of
 * vertex positions in this list must be identical to the order
 * of indices assigned to Vertex::newPosition, Edge::newPosition,
 * and Face::newPosition.
 */
void HalfedgeMesh::buildSubdivisionVertexList(vector<Vector3D> &subDVertices) {
  // TODO Resize the vertex list so that it can hold all the vertices.
  subDVertices.resize(vertices.size() + edges.size() + faces.size());
  // TODO Iterate over vertices, assigning Vertex::newPosition to the
  // appropriate location in the new vertex list.
  Index idx = 0;
  for (auto &vertex : vertices) {
    subDVertices[idx] = vertex.newPosition;
    idx++;
  }
  // TODO Iterate over edges, assigning Edge::newPosition to the appropriate
  // location in the new vertex list.
  for (auto &edge : edges) {
    subDVertices[idx] = edge.newPosition;
    idx++;
  }

  // TODO Iterate over faces, assigning Face::newPosition to the appropriate
  // location in the new vertex list.
  for (auto &face : faces) {
    subDVertices[idx] = face.newPosition;
    idx++;
  }
}

/**
 * Build a flat list containing all the quads in a Catmull-Clark
 * (or linear) subdivision of this mesh.  Each quad is specified
 * by a vector of four indices (i,j,k,l), which come from the
 * members Vertex::index, Edge::index, and Face::index.  Note that
 * the ordering of these indices is important because it determines
 * the orientation of the new quads; it is also important to avoid
 * "bowties."  For instance, (l,k,j,i) has the opposite orientation
 * of (i,j,k,l), and if (i,j,k,l) is a proper quad, then (i,k,j,l)
 * will look like a bowtie.
 */
void HalfedgeMesh::buildSubdivisionFaceList(vector<vector<Index>> &subDFaces) {
  // TODO This routine is perhaps the most tricky step in the construction of
  // a subdivision mesh (second, perhaps, to computing the actual Catmull-Clark
  // vertex positions).  Basically what you want to do is iterate over faces,
  // then for each for each face, append N quads to the list (where N is the
  // degree of the face).  For this routine, it may be more convenient to simply
  // append quads to the end of the list (rather than allocating it ahead of
  // time), though YMMV.  You can of course iterate around a face by starting
  // with its first halfedge and following the "next" pointer until you get
  // back to the beginning.  The tricky part is making sure you grab the right
  // indices in the right order---remember that there are indices on vertices,
  // edges, AND faces of the original mesh.  All of these should get used.  Also
  // remember that you must have FOUR indices per face, since you are making a
  // QUAD mesh!

  // TODO iterate over faces
  // TODO loop around face
  // TODO build lists of four indices for each sub-quad
  // TODO append each list of four indices to face list
  for (auto &face : faces) {
    auto h = face.halfedge();
    do {
      vector<Index> indices;
      indices.push_back(face.index);
      indices.push_back(h->edge()->index);
      indices.push_back(h->twin()->vertex()->index);
      indices.push_back(h->next()->edge()->index);

      subDFaces.push_back(indices);
      h = h->next();
    } while (h != face.halfedge());
  }
}

FaceIter HalfedgeMesh::bevelVertex(VertexIter v) {
  // TODO This method should replace the vertex v with a face, corresponding to
  // a bevel operation. It should return the new face.  NOTE: This method is
  // responsible for updating the *connectivity* of the mesh only---it does not
  // need to update the vertex positions.  These positions will be updated in
  // HalfedgeMesh::bevelVertexComputeNewPositions (which you also have to
  // implement!)

  showError("bevelVertex() not implemented.");
  return facesBegin();
}

FaceIter HalfedgeMesh::bevelEdge(EdgeIter e) {
  // TODO This method should replace the edge e with a face, corresponding to a
  // bevel operation. It should return the new face.  NOTE: This method is
  // responsible for updating the *connectivity* of the mesh only---it does not
  // need to update the vertex positions.  These positions will be updated in
  // HalfedgeMesh::bevelEdgeComputeNewPositions (which you also have to
  // implement!)

  showError("bevelEdge() not implemented.");
  return facesBegin();
}

FaceIter HalfedgeMesh::bevelFace(FaceIter f) {
  // TODO This method should replace the face f with an additional, inset face
  // (and ring of faces around it), corresponding to a bevel operation. It
  // should return the new face.  NOTE: This method is responsible for updating
  // the *connectivity* of the mesh only---it does not need to update the vertex
  // positions.  These positions will be updated in
  // HalfedgeMesh::bevelFaceComputeNewPositions (which you also have to
  // implement!)

  showError("bevelFace() not implemented.");
  return facesBegin();
}

void HalfedgeMesh::bevelFaceComputeNewPositions(
    vector<Vector3D> &originalVertexPositions,
    vector<HalfedgeIter> &newHalfedges, double normalShift,
    double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled face.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., newHalfedges.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the originalVertexPositions array) to compute an offset vertex
  // position.
  //
  // Note that there is a 1-to-1 correspondence between halfedges in
  // newHalfedges and vertex positions
  // in orig.  So, you can write loops of the form
  //
  // for( int i = 0; i < newHalfedges.size(); hs++ )
  // {
  //    Vector3D pi = originalVertexPositions[i]; // get the original vertex
  //    position correponding to vertex i
  // }
  //
}

void HalfedgeMesh::bevelVertexComputeNewPositions(
    Vector3D originalVertexPosition, vector<HalfedgeIter> &newHalfedges,
    double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled vertex.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., hs.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the orig array) to compute an offset vertex position.
}

void HalfedgeMesh::bevelEdgeComputeNewPositions(
    vector<Vector3D> &originalVertexPositions,
    vector<HalfedgeIter> &newHalfedges, double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled edge.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., newHalfedges.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the orig array) to compute an offset vertex position.
  //
  // Note that there is a 1-to-1 correspondence between halfedges in
  // newHalfedges and vertex positions
  // in orig.  So, you can write loops of the form
  //
  // for( int i = 0; i < newHalfedges.size(); i++ )
  // {
  //    Vector3D pi = originalVertexPositions[i]; // get the original vertex
  //    position correponding to vertex i
  // }
  //
}

void HalfedgeMesh::splitPolygons(vector<FaceIter> &fcs) {
  for (auto f : fcs)
    splitPolygon(f);
}

void HalfedgeMesh::splitPolygon(FaceIter f) {
  // TODO: (meshedit)
  // Triangulate a polygonal face
  //  cout << "I'm here! " << f->degree() << endl;

  // Get vertices.
  std::vector<VertexIter> v_list;
  std::vector<EdgeIter> e_list;
  std::vector<HalfedgeIter> he_list;
  HalfedgeIter h = f->halfedge();
  do {
    v_list.push_back(h->vertex());
    e_list.push_back(h->edge());
    he_list.push_back(h);
    h = h->next();
  } while (h != f->halfedge());

  const int nv = v_list.size();
  if (nv == 3) {
    return;
  }

  FaceIter current_face = f;
  HalfedgeIter current_he = f->halfedge();

  HalfedgeIter h0n, h1n;
  for (int i = 2; i < nv - 1; i++) {
    auto en = newEdge();
    auto fn = newFace();
    h0n = newHalfedge();
    h1n = newHalfedge();

    h0n->twin() = h1n;
    h0n->vertex() = v_list[i];
    h0n->edge() = en;
    h0n->face() = current_face;
    h0n->next() = current_he;

    h1n->twin() = h0n;
    h1n->vertex() = v_list[0];
    h1n->edge() = en;
    h1n->face() = fn;
    h1n->next() = he_list[i];

    en->halfedge() = h0n;
    fn->halfedge() = h1n;
    he_list[i - 1]->next() = h0n;
    he_list[nv - 1]->next() = h1n;

    current_face = fn;
    current_he = h1n;
  }
}

EdgeRecord::EdgeRecord(EdgeIter &_edge) : edge(_edge) {
  // TODO: (meshEdit)
  // Compute the combined quadric from the edge endpoints.
  // -> Build the 3x3 linear system whose solution minimizes the quadric error
  //    associated with these two endpoints.
  // -> Use this system to solve for the optimal position, and store it in
  //    EdgeRecord::optimalPoint.
  // -> Also store the cost associated with collapsing this edg in
  //    EdgeRecord::Cost.
  auto v0 = edge->halfedge()->vertex();
  auto v1 = edge->halfedge()->twin()->vertex();
  Matrix3x3 B;
  B.zero();
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      B(i, j) += v0->quadric(i, j);
      B(i, j) += v1->quadric(i, j);
    }
  }

  Vector3D w;
  for(int i = 0; i < 3; i++) {
    w[i] += v0->quadric(i, 3);
    w[i] += v1->quadric(i, 3);
  }

  double d2 = v0->quadric(3, 3) + v1->quadric(3, 3);

  auto x = -B.inv() * w;
  optimalPoint = x;
  score = dot(x, B * x) + 2 * dot(w, x) + d2;
//  cout << v0->quadric + v1->quadric << endl;
//  cout << B << endl;
//  cout << w << endl;
//  cout << "v0: " << v0->position << endl;
//  cout << "v1: " << v1->position << endl;
//  cout << "x: " << x << endl;
//  cout << "B_det: " << B.det() << endl;
//  cout << "score: " << score << endl << endl;
}

void MeshResampler::upsample(HalfedgeMesh &mesh)
// This routine should increase the number of triangles in the mesh using Loop
// subdivision.
{
  // TODO: (meshEdit)
  // Compute new positions for all the vertices in the input mesh, using
  // the Loop subdivision rule, and store them in Vertex::newPosition.
  // -> At this point, we also want to mark each vertex as being a vertex of the
  //    original mesh.
  // -> Next, compute the updated vertex positions associated with edges, and
  //    store it in Edge::newPosition.
  // -> Next, we're going to split every edge in the mesh, in any order.  For
  //    future reference, we're also going to store some information about which
  //    subdivided edges come from splitting an edge in the original mesh, and
  //    which edges are new, by setting the flat Edge::isNew. Note that in this
  //    loop, we only want to iterate over edges of the original mesh.
  //    Otherwise, we'll end up splitting edges that we just split (and the
  //    loop will never end!)
  // -> Now flip any new edge that connects an old and new vertex.
  // -> Finally, copy the new vertex positions into final Vertex::position.

  // Each vertex and edge of the original surface can be associated with a
  // vertex in the new (subdivided) surface.
  // Therefore, our strategy for computing the subdivided vertex locations is to
  // *first* compute the new positions
  // using the connectity of the original (coarse) mesh; navigating this mesh
  // will be much easier than navigating
  // the new subdivided (fine) mesh, which has more elements to traverse.  We
  // will then assign vertex positions in
  // the new mesh based on the values we computed for the original mesh.

  // Compute updated positions for all the vertices in the original mesh, using
  // the Loop subdivision rule.

  // Next, compute the updated vertex positions associated with edges.

  // Next, we're going to split every edge in the mesh, in any order.  For
  // future
  // reference, we're also going to store some information about which
  // subdivided
  // edges come from splitting an edge in the original mesh, and which edges are
  // new.
  // In this loop, we only want to iterate over edges of the original
  // mesh---otherwise,
  // we'll end up splitting edges that we just split (and the loop will never
  // end!)

  // Finally, flip any new edge that connects an old and new vertex.

  // Copy the updated vertex positions to the subdivided mesh.
  showError("upsample() not implemented.");
}

void MeshResampler::downsample(HalfedgeMesh &mesh) {
  // TODO: (meshEdit)
  // Compute initial quadrics for each face by simply writing the plane equation
  // for the face in homogeneous coordinates. These quadrics should be stored
  // in Face::quadric
  // -> Compute an initial quadric for each vertex as the sum of the quadrics
  //    associated with the incident faces, storing it in Vertex::quadric
  // -> Build a priority queue of edges according to their quadric error cost,
  //    i.e., by building an EdgeRecord for each edge and sticking it in the
  //    queue.
  // -> Until we reach the target edge budget, collapse the best edge. Remember
  //    to remove from the queue any edge that touches the collapsing edge
  //    BEFORE it gets collapsed, and add back into the queue any edge touching
  //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
  //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
  //    top of the queue.

  int triangle_count = 0;
  for(auto face = mesh.facesBegin(); face != mesh.facesEnd(); face++) {
    const Vector3D& p = face->halfedge()->vertex()->position;
    Vector3D n = face->normal();
    Vector4D v(n, -dot(p, n));
    face->quadric = outer(v, v);
    triangle_count++;
  }

  for (auto vertex = mesh.verticesBegin(); vertex != mesh.verticesEnd();
       vertex++) {
    vertex->quadric.zero();
    auto v_flist = findFaceForVertex(vertex);
    for(auto& f : v_flist) {
      vertex->quadric += f->quadric;
    }
  }

  MutablePriorityQueue<EdgeRecord> queue;
  for (auto edge = mesh.edgesBegin(); edge != mesh.edgesEnd(); edge++) {
    edge->record = EdgeRecord(edge);
    queue.insert(edge->record);
  }

  int current_triangle_count = triangle_count;
  while(current_triangle_count > 0.25 * triangle_count) {
    // 1. Get the cheapest edge from the queue.
    const EdgeRecord er = queue.top();
    auto e = er.edge;
    auto v0 = e->halfedge()->vertex();
    auto v1 = e->halfedge()->twin()->vertex();

    // 2. Remove the cheapest edge from the queue by calling pop().
    queue.pop();

    // 3. Compute the new quadric by summing the quadrics at its two endpoints.
    Matrix4x4 K = v0->quadric + v1->quadric;

    // 4. Remove any edge touching either of its endpoints from the queue.
    vector<vector<HalfedgeIter>> h_lists {
        findHalfedgesForVertex(e->halfedge()),
        findHalfedgesForVertex(e->halfedge()->twin())};

    vector<EdgeIter> e_list;
    for(auto& h_list : h_lists) {
      for(auto h = h_list.begin() +1 ; h!= h_list.end() ;h++) {
        e_list.push_back((*h)->edge());
      }
    }

    for(auto& edge : e_list) {
      queue.remove(edge->record);
    }

    // 5. Collapse the edge.
    auto v = mesh.collapseEdge(e);

    // 6. Set the quadric of the new vertex to the quadric computed in Step 3.
    v->quadric = K;
    v->position = er.optimalPoint;

    // 7. Insert any edge touching the new vertex into the queue.
    cout << v->degree() << endl;
    cout << &(*v) << endl;
    cout << v->position << endl;
    cout << er.optimalPoint << endl;
    cout << er.score << endl << endl;

    auto h_list = findHalfedgesForVertex(v->halfedge());
    for(auto& h : h_list) {
      auto edge = h->edge();
      edge->record = EdgeRecord(edge);
      queue.insert(edge->record);
    }

    current_triangle_count -= 2;
  }


}

void MeshResampler::resample(HalfedgeMesh &mesh) {
  // TODO: (meshEdit)
  // Compute the mean edge length.
  // Repeat the four main steps for 5 or 6 iterations
  // -> Split edges much longer than the target length (being careful about
  //    how the loop is written!)
  // -> Collapse edges much shorter than the target length.  Here we need to
  //    be EXTRA careful about advancing the loop, because many edges may have
  //    been destroyed by a collapse (which ones?)
  // -> Now flip each edge if it improves vertex degree
  // -> Finally, apply some tangential smoothing to the vertex positions
  showError("resample() not implemented.");
}

} // namespace CMU462
