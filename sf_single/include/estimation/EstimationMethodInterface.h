/**
 * @file 
 * @author Denise Ratasich
 * @date 28.08.2013
 *
 * @brief Interface of an estimation method (template).
 */

#ifndef __ESTIMATION_ESTIMATIONMETHODINTERFACE_H__
#define __ESTIMATION_ESTIMATIONMETHODINTERFACE_H__

#include <vector>
#include <stdexcept>

namespace estimation 
{

  /**
   * @brief Interface of an estimation method.
   *
   * An estimation method has input and output entities. Entities of
   * the same type for an estimation method can be collected with this
   * class template. The order of the entities does not change,
   * i.e. the entities are identifiable within this interface through
   * its position. The order is specified at initialization (order in
   * vector). Adding a new entity will append it to the collection.
   */
  template <class T>
  class EstimationMethodInterface
  {
    /** @brief Stores the entities. */
    std::vector<T> entities;

  public:
    /**
     * @brief Basic Constructor.
     *
     * Empty. No initializations. No entities.
     */
    EstimationMethodInterface();

    /**
     * @brief Constructor of this interface which adds a single
     * entity.
     *
     * Use it when your estimation algorithm works only with a single
     * entity.
     *
     * @param entity An entity.
     */
    EstimationMethodInterface(T entity);

    /**
     * @brief Constructor of this class which adds several entities of
     * an estimation algorithm to this interface.
     *
     * E.g. use it when your estimation algorithm is able to estimate
     * more than one entity.
     *
     * @param entities Several entities for this interface.
     */
    EstimationMethodInterface(std::vector<T> entities);

    // -----------------------------------------
    // manipulation of this interface
    // -----------------------------------------
    /**
     * @brief Append another entity to this interface.
     */
    void add(T entity);

    // -----------------------------------------
    // getters and setters
    // -----------------------------------------
    /**
     * @brief Returns the values of the entities.
     *
     * Collects the values of the entities and returns them in a
     * vector. The order of the values corresponds to the order of the
     * entities in the interface.
     *
     * @return The values of the entities.
     */
    std::vector<double> getValues(void) const;

    /**
     * @brief Returns the value of the first entity.
     *
     * You can use it if you have only a single entity in this
     * interface, e.g. when using an estimation algorithm where only
     * one entity is estimated.
     */
    double getValue() const;

    /**
     * @brief Returns the number of entities collected in this
     * interface.
     */
    int size() const;

    // -----------------------------------------
    // overloading operators
    // -----------------------------------------
    /**
     * @brief Swaps the data of two elements.
     */
    void swap(EstimationMethodInterface& first, EstimationMethodInterface& second);

    /**
     * @brief Overloads the assign operator.
     */
    EstimationMethodInterface& operator=(EstimationMethodInterface right);


    /**
     * @brief Overloads the subscript operator to use this class like
     * an array.
     *
     * Throws an out_of_range exception if the index is invalid.
     */
    T& operator[](const int index);

    // remove if really not needed: == !=
    // /**
    //  * @brief Overloads the == (equal) operator (compares the values).
    //  */
    // bool operator==(const EstimationMethodInterface& rhs) const;

    // /**
    //  * @brief Overloads the != (not-equal) operator (compares the
    //  * values).
    //  */
    // bool operator!=(const EstimationMethodInterface& rhs) const;
  };

  ////////////////////
  // Implementation //
  ////////////////////
  template <class T>
  EstimationMethodInterface<T>::EstimationMethodInterface() { }

  template <class T>
  EstimationMethodInterface<T>::EstimationMethodInterface(T entity)
  {
    entities.push_back(entity);	// append
  }

  template <class T>
  EstimationMethodInterface<T>::EstimationMethodInterface(std::vector<T> entities)
  {
    this->entities = entities;	// copy
  }

  // -----------------------------------------
  // manipulation of this interface
  // -----------------------------------------
  template <class T>
  void EstimationMethodInterface<T>::add(T entity)
  {
    entities.push_back(entity);	// append
  }

  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  template <class T>
  std::vector<double> EstimationMethodInterface<T>::getValues(void) const
  {
    std::vector<double> values;

    for (int i = 0; i < entities.size(); i++)
      values.push_back(entities[i].getValue());

    return values;
  }

  template <class T>
  double EstimationMethodInterface<T>::getValue() const
  {
    return entities[0].getValue();
  }

  template <class T>
  int EstimationMethodInterface<T>::size() const
  {
    return entities.size();
  }

  // -----------------------------------------
  // overloading operators
  // -----------------------------------------
  template <class T>
  void EstimationMethodInterface<T>::swap(EstimationMethodInterface<T>& first, EstimationMethodInterface<T>& second)
  {
    // enable ADL (not necessary in our case, but good practice)
    using std::swap; 

    // by swapping the members of two classes,
    // the two classes are effectively swapped
    swap(first.entities, second.entities);
  }

  template <class T>
  EstimationMethodInterface<T>& EstimationMethodInterface<T>::operator=(EstimationMethodInterface<T> right)
  {
    swap(*this, right);
    return *this;
  }

  template <class T>
  T& EstimationMethodInterface<T>::operator[](const int index)
  {
    if (index < 0  ||  index >= entities.size())
      throw std::out_of_range("Index out of range (Output.data).");

    return entities[index];
  }

  // remove if really not needed: == !=
  // bool EstimationMethodInterface::operator==(const EstimationMethodInterface& rhs) const
  // {
  //   bool equal = true;

  //   for (int i = 0; i < (*this).data.size(); i++) {
  //     if ((*this).data[i] != rhs.data[i]) {
  // 	equal = false;
  // 	break;
  //     }
  //   }

  //   return equal;
  // }

  // bool EstimationMethodInterface::operator!=(const EstimationMethodInterface& rhs) const
  // {
  //   return !(*this).operator==(rhs);
  // }

} // end namespace "estimation"

#endif
