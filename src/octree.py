from collections import namedtuple

import numpy as np
from scipy.spatial import cKDTree as KDTree


# TODO (masasin):
#     - Allow dynamic expansion of Octree
#     - Merge empty sibling leaves


Coordinates = namedtuple("Coordinates", "x y z")


class OctreeError(Exception):
    """Generic octree error."""


class OctreeBoundsError(OctreeError):
    """Raised when a point is inserted outside of the bounds of the octree."""


class _Frame(object):
    """
    Dummy frame class.

    In a real application, it might normally include the pose at which the image
    was taken (perhaps as a quaternion), its timestamp, a thumbnail, and the
    full image. A quality score might also be added, based on blur.

    Parameters
    ----------
    position : ndarray
        The x, y, z coordinates at which the frame was taken.
    contents : any
        Any content.

    Attributes
    ----------
    position : ndarray
        The x, y, z coordinates at which the frame was taken.
    contents : any
        Any content.

    """
    def __init__(self, position, contents):
        self.position = np.asarray(position)
        self.contents = contents

    def __repr__(self):
        return "Frame ({}): {}".format(self.position.tolist(), self.contents)


class Data(object):
    """
    A data class.

    It represents the data value of a given node. If there is no data, the
    position field is also cleared.

    Attributes
    ----------
    is_empty
    position : ndarray
        The x, y, z position of the data stored at the node.
    contents : list of _Frame
        A list of all frames added to the octree at this point, in the order
        they were added. A full implementation might sort the frames by
        quality.

    """
    def __init__(self):
        self.position = None
        self.contents = []

    def __len__(self):
        """The length of the `contents` field."""
        return len(self.contents)

    def __repr__(self):
        return "Data ({}): {}".format(self.position, self.contents)

    @property
    def is_empty(self):
        """
        Check whether the data has no content.

        Returns
        -------
        bool
            True if the data has no content.

        """
        return not len(self.contents)

    def append(self, item):
        """
        Append an item to `contents`.

        Parameters
        ----------
        item : _Frame
            Any item with a position attribute.

        """
        if self.position is None:
            self.position = np.asarray(item.position)
        elif any(item.position != self.position):
            raise OctreeError("Wrong position")
        self.contents.append(item)

    def pop(self):
        """Return and remove the last item in `contents`."""
        return self.contents.pop()

    def clear(self):
        """Reset the `Data` object to its default state."""
        self.position = None
        self.contents = []


class Octree(object):
    """
    An octree class.

    In this implementation, octrees are perfectly cubical, and they can only
    accept one data point. If more than one data point is inserted, the octree
    splits. If an item is removed, and that node's siblings have no data either,
    it currently does not merge those nodes to form the parent node.

    Changes to the boundaries, the centre, or the length update the other
    variables simultaneously.

    Parameters
    ----------
    centre : ndarray
        The x, y, z coordinates of the centre of the octree.
    half_dim : float
        Half the length of one side of the cube.
    parent : Octree, optional
        The parent of the octree. Default (for the root node) is None.

    Attributes
    ----------
    centre
    half_dim
    side
    bound_min
    bound_max
    parent : Octree
        The parent of the octree. Root nodes have a parent of None.
    children: list of Octree
        The list of children, one at each octant. The binary value of the list
        index represents the x, y, and z coordinates respectively.  If a
        coordinate's value is less than the centre's, its index will have a
        value of "0"; coordinates with a value greater than or equal to the
        centre's are displayed as "1".
    data : Data
        The data contained in the octree.

    """
    def __init__(self, centre, half_dim, parent=None):
        self.parent = parent

        # Store a set of points for easy retrieval. Can be converted into a
        # numpy array in order to find nearest neighbours.
        if self.parent is not None:  # Not the root node
            self._root = self.parent._root
            self._points = self._root._points
            self._points_array = self._root._points_array
            self._points_array_up_to_date = self._root._points_array_up_to_date
        else:  # Root node
            self._root = self
            self._points = set()
            self._points_array = None
            self._points_array_up_to_date = True

        self._centre = np.asarray(centre)
        self._half_dim = half_dim
        self._bound_min = self.centre - self.half_dim * np.ones(3)
        self._bound_max = self.centre + self.half_dim * np.ones(3)
        self._n_items = 0

        self.children = [None] * 8
        self.data = Data()

    def __len__(self):
        """Return the number of items in the subtree."""
        return self._n_items

    def __repr__(self):
        return "[{}D{}C{}I]".format(0 if self.data.is_empty else 1,
                                    0 if self._is_leaf_node() else 1,
                                    self._n_items)

    def _get_octant(self, point):
        """
        Return the octant that the point is in.

        Parameters
        ----------
        point : ndarray
            The x, y, z coordinates of the point whose quadrant is to be found.

        Returns
        -------
        int
            The octant that the point is in.

        """
        pos = point >= self.centre
        return int("".join("1" if i else "0" for i in pos), base=2)

    def _is_leaf_node(self):
        """
        Check whether the node is a leaf node.

        A leaf node does not have any children, and may contain data. Since the
        children are either all None or all nodes, we can check only the first
        child.

        Returns
        -------
        bool
            True if the node is a leaf node.

        """
        return self.children[0] is None

    def _point_in_node(self, point):
        """
        Check whether a point's coordinates are within the node boundaries.

        Returns
        -------
        bool
            True if the point is within the bounding box.

        """
        return self._point_in_box(point, self.bound_min, self.bound_max)

    def _point_in_box(self, point, bound_min, bound_max):
        """
        Check whether a point's coordinates are within a box boundaries.

        Returns
        -------
        bool
            True if the point is within the bounding box.

        """
        return all(point >= bound_min) and all(point <= bound_max)

    def _node_outside_box(self, node, bound_min, bound_max):
        """
        Check whether a node is completely outside a box boundaries.

        Returns
        -------
        bool
            True if a node is completely outside the box boundaries.

        """
        node_bmin = node.bound_min
        node_bmax = node.bound_max
        return all(node_bmin > bound_max) or all(node_bmax < bound_min)

    def insert(self, item):
        """
        Insert an item into the tree.

        The item is inserted into a relevant leaf node; it splits the node as
        necessary in order to maintain the constraint of one position per node.

        Parameters
        ----------
        item : _Frame
            The item to be inserted. The item must have a position attribute.

        Raises
        ------
        OctreeBoundsError
            If the item coordinates are outside the boundaries of the octree.

        Notes
        -----
        This algorithm is recursive. Whenever it enters a new node, it calls
        the function again.

        """
        if not self._point_in_node(item.position) and self.parent is None:
            raise OctreeBoundsError

        if self._is_leaf_node():
            # Register the point
            self._points.add(Coordinates(*item.position))
            if self._points_array_up_to_date:
                self._points_array_up_to_date = False

            # Data can be added.
            if self.data.is_empty or all(self.data.position == item.position):
                # A node may only have data at one position.
                self.data.append(item)
            else:
                # Split the node by creating new children.
                new_half_dim = self.half_dim / 2
                for pos in range(len(self.children)):
                    new_centre = self.centre.copy()
                    for i, sign in enumerate(bin(pos)[2:]):
                        multiplier = 1 if sign == "1" else -1
                        new_centre[i] += new_half_dim * multiplier
                    self.children[pos] = Octree(new_centre,
                                                new_half_dim,
                                                parent=self)

                # The old data contents must be moved into a child node.
                old_data_octant = self._get_octant(self.data.position)
                self.children[old_data_octant].extend(self.data.contents)
                self.data.clear()

                # The new data is inserted recursively into the child node.
                new_data_octant = self._get_octant(item.position)
                self.children[new_data_octant].insert(item)

        else:
            # This is an interior node. We need to go the leaf.
            octant = self._get_octant(item.position)
            self.children[octant].insert(item)

        self._n_items += 1

    def extend(self, items):
        """
        Extend the octree with a sequence of items.

        This function inserts each item in the sequence into its respective node
        on the tree.

        Parameters
        ----------
        items : list of _Frame
            An iterable of items to be added.

        Raises
        ------
        OctreeBoundsError
            If at least one item in the list is outside the bounding box of the
            octree.

        """
        for item in items:
            self.insert(item)

    def _get(self, point):
        """
        Retrieve the data at a given point.

        Parameters
        ----------
        point : ndarray
            The x, y, z coordinates of the point whose data is to be retrieved.

        Returns
        -------
        data : Data
            The data requested.

        Raises
        ------
        KeyError
            If the data does not exist.

        Notes
        -----
        This method is recursive.

        """
        if Coordinates(*point) not in self._points:
            print(point, self._points)
            raise KeyError("Could not find point.")

        if self._is_leaf_node():
            return self.data

        octant = self._get_octant(point)
        return self.children[octant]._get(point)

    def get(self, points):
        """
        Get a sequence of items.

        Parameters
        ----------
        point : iterable of ndarray
            A list of the coordinates of the points whose data is to be
            retrieved.

        Returns
        -------
        Data or list of Data
            The data requested. If only a single point is requested, the data
            will not be added to the list.

        Raises
        ------
        KeyError
            If any point does not exist.

        """
        try:
            points[0][0]  # Check to see if list of lists. Assumes no strings.
            data = []
            for point in points:
                data.append(self._get(point))
        except (TypeError, IndexError):
            data = self._get(points)

        return data

    def remove(self, point, clear=False):
        """
        Remove the last item from a point.

        Parameters
        ----------
        point : ndarray
            The x, y, z coordinates of the point to be removed.
        clear : bool, optional
            If True, empties the contents of the data at the point. Default is
            False.

        Raises
        ------
        KeyError
            If the data does not exist.

        """
        data = self.get(point)
        node = self

        if clear:
            n_cleared = len(data.contents)
            self._points.remove(Coordinates(*data.position))
            if self._points_array_up_to_date:
                self._points_array_up_to_date = False
            data.clear()

            while node is not None:
                node._n_items -= n_cleared
                node = node.parent
        else:
            data.pop()
            if data.is_empty:
                self._points.remove(Coordinates(*data.position))
                if self._points_array_up_to_date:
                    self._points_array_up_to_date = False
                data.clear()
            while node is not None:
                self._n_items -= 1
                node = node.parent

    def get_points_in_box(self, bound_min, bound_max):
        """
        Get all the points in a given box.

        Parameters
        ----------
        bound_min : ndarray
            The x, y, z coordinates of the vertex with the minimum values.
        bound_max : ndarray
            The x, y, z coordinates of the vertex with the maximum values.

        Yields
        ------
        Data
            The data at the points that fall within the bounding box.

        Notes
        -----
        This method is recursive.

        """
        if self._is_leaf_node():
            if not self.data.is_empty:
                if self._point_in_box(self.data.position, bound_min, bound_max):
                    yield self.data
        else:
            for child in self.children:
                if self._node_outside_box(child, bound_min, bound_max):
                    continue
                for point in child.get_points_in_box(bound_min, bound_max):
                    yield point

    def get_nearest(self, point, k=1):
        """
        Get the k points nearest to a given point.

        Note that if the point corresponds to an existing coordinate, then the
        distance is zero, and the data at that point will also be returned.

        Parameters
        ----------
        point : ndarray
            The x, y, z coordinates of the point.
        k : int
            The number of neighbours to find.

        Returns
        -------
        distances : list of float
            A list of the distance to the point of each of the respective
            `nearest` data points.
        nearest : list of Data
            A sorted list of the data at the nearest populated locations.

        Raises
        ------
        ValueError
            - If there are no filled nodes in the octree.
            - If k has an invalid value.

        """
        if k < 1:
            raise ValueError("At least one neighbour needs to be found.")

        if not self._points_array_up_to_date:
            self._points_array = np.array(list(self._points))
            self._points_array_up_to_date = True

        tree = KDTree(self._points_array)
        distances, indices = tree.query(point, k=min(k, len(tree.data)))

        if k == 1:
            distances = [distances]
            indices = [indices]

        nearest = []
        for index in indices:
            nearest.append(self.get(tree.data[index]))

        return distances, nearest

    @property
    def centre(self):
        """
        Return the x, y, z coordinates of the centre of the octree.

        Returns
        -------
        ndarray
            The x, y, z coordinates of the centre of the octree.

        """
        return self._centre

    @centre.setter
    def centre(self, values):
        """
        Move the x, y, z coordinates of the centre of the octree.

        Update the bounds simultaneously.

        Parameters
        ----------
        values : ndarray
            The coordinates of the centre.

        """
        self._centre = np.asarray(values)
        self._bound_min = self.centre - self.half_dim * np.ones(3)
        self._bound_max = self.centre + self.half_dim * np.ones(3)

    @property
    def half_dim(self):
        """
        Return half the length of one side of the cube.

        Returns
        -------
        float
            Half the length of one side of the cube.

        """
        return self._half_dim

    @half_dim.setter
    def half_dim(self, value):
        """
        Set the length of half one side of the cube, while keeping the centre.

        Update the bounds simultaneously.

        Parameters
        ----------
        value : float
            The new length of half of one side.

        """
        self._half_dim = value
        self._bound_min = self.centre - self.half_dim * np.ones(3)
        self._bound_max = self.centre + self.half_dim * np.ones(3)

    @property
    def side(self):
        """
        Return the length of one side of the octree.

        Returns
        -------
        float
            The length of one side of the octree.

        """
        return self.half_dim * 2

    @property
    def bound_min(self):
        """
        Return the coordinates of the vertex with the minimum values.

        Returns
        -------
        ndarray
            The x, y, z coordinates of the vertex with the minimum values.

        """
        return self._bound_min

    @bound_min.setter
    def bound_min(self, values):
        """
        Set the minimum boundary of the octree while keeping the maximum.

        Update the centre and half_dim lengths simultaneously.

        Parameters
        ----------
        values : ndarray
            The x, y, z coordinates of the vertex with the minimum values.

        """
        self._bound_min = np.asarray(values)
        self._centre = (self.bound_max + self.bound_min) / 2
        self._half_dim = (self.bound_min - self.bound_min) / 2

    @property
    def bound_max(self):
        """
        Return the coordinates of the vertex with the maximum values.

        Returns
        -------
        ndarray
            The x, y, z coordinates of the vertex with the maximum values.

        """
        return self._bound_max

    @bound_max.setter
    def bound_max(self, values):
        """
        Set the maximum boundary of the octree while keeping the minimum.

        Update the centre and half_dim lengths simultaneously.

        Parameters
        ----------
        values : ndarray
            The x, y, z coordinates of the vertex with the maximum values.

        """
        self._bound_max = np.asarray(values)
        self._centre = (self.bound_max + self.bound_min) / 2
        self._half_dim = (self.bound_min - self.bound_min) / 2


if __name__ == "__main__":
    o = Octree((0, 0, 0), 100)
    item1 = _Frame((20, 30, 40), "An item 1")
    item1_copy = _Frame((20, 30, 40), "An item 1 copy")
    item2 = _Frame((30, 30, 40), "An item 2")
    item3 = _Frame((40, 30, 40), "An item 3")

    o.insert(item1)
    o.insert(item1_copy)
    o.insert(item2)
    o.insert(item3)
    print(list(o.get_points_in_box((0, 0, 0), (25, 35, 50))))
