import pytest

from ntree import Octree, NtreeBoundsError, Frame


# Note that Ntrees with n_dims != 3 have not been tested yet.


class TestInitialization(object):
    def setup(self):
        self.o = Octree((0, 0, 0), 100)

    def test_default_limits(self):
        self.o.bound_min.tolist() == [-100, -100, -100]
        self.o.bound_max.tolist() == [100, 100, 100]


class TestInsertion(object):
    def setup(self):
        self.o = Octree((0, 0, 0), 100)
        self.item1 = Frame((20, 30, 40), "An item 1")
        self.item1_copy = Frame((20, 30, 40), "An item 1 copy")
        self.item2 = Frame((30, 30, 40), "An item 2")
        self.item3 = Frame((40, 30, 40), "An item 3")

    def test_add_item_out_of_bounds(self):
        item = Frame((20, 110, 40), "An item")
        with pytest.raises(NtreeBoundsError):
            self.o.insert(item)

    def test_add_one_item(self):
        self.o.insert(self.item1)
        assert self.o.data.contents == [self.item1]

    def test_count_increments(self):
        self.o.insert(self.item1)
        assert len(self.o) == 1, "Item count not incremented"

    def test_add_multiple_items(self):
        self.o.insert(self.item1)
        self.o.insert(self.item2)
        self.o.insert(self.item3)
        assert len(self.o) == 3, "Item count not incremented"

    def test_extend(self):
        self.o.extend([self.item1, self.item2, self.item3])
        assert len(self.o) == 3, "Item count not incremented"

    def test_add_items_same_place(self):
        self.o.insert(self.item1)
        self.o.insert(self.item1_copy)
        assert self.o.data.contents == [self.item1, self.item1_copy]


class TestGetting(object):
    def setup(self):
        self.o = Octree((0, 0, 0), 100)
        self.item1 = Frame((20, 30, 40), "An item 1")
        self.item1_copy = Frame((20, 30, 40), "An item 1 copy")
        self.item2 = Frame((30, 30, 40), "An item 2")
        self.item3 = Frame((40, 30, 40), "An item 3")

    def test_get_empty_tree(self):
        with pytest.raises(KeyError):
            self.o.get((20, 30, 40))

    def test_get_nonexistent_item(self):
        self.o.insert(self.item1)
        self.o.insert(self.item1_copy)
        self.o.insert(self.item2)
        self.o.insert(self.item3)
        with pytest.raises(KeyError):
            self.o.get((20, 90, 40))

    def test_get_existing_item(self):
        self.o.insert(self.item1)
        self.o.insert(self.item1_copy)
        self.o.insert(self.item2)
        self.o.insert(self.item3)
        data = self.o.get((40, 30, 40))
        assert data.contents == [self.item3]

    def test_get_existing_multiple_item(self):
        self.o.insert(self.item1)
        self.o.insert(self.item1_copy)
        self.o.insert(self.item2)
        self.o.insert(self.item3)
        data = self.o.get([(40, 30, 40), [30, 30, 40]])
        assert data[0].contents == [self.item3]
        assert data[1].contents == [self.item2]

    def test_get_item_pair(self):
        self.o.insert(self.item1)
        self.o.insert(self.item1_copy)
        data = self.o.get((20, 30, 40))
        assert data.contents == [self.item1, self.item1_copy]

    def test_get_existing_item_list(self):
        self.o.insert(self.item1)
        self.o.insert(self.item1_copy)
        self.o.insert(self.item2)
        self.o.insert(self.item3)
        data = self.o.get((20, 30, 40))
        assert data.contents == [self.item1, self.item1_copy]

    def test_get_existing_item_list_after_split(self):
        self.o.insert(self.item1)
        self.o.insert(self.item2)
        self.o.insert(self.item3)
        self.o.insert(self.item1_copy)
        data = self.o.get((20, 30, 40))
        assert data.contents == [self.item1, self.item1_copy]

    def test_get_nearest_empty(self):
        with pytest.raises(ValueError):
            dists, data = self.o.get_nearest((26, 30, 40))

    def test_get_nearest_point_arbitrary(self):
        self.o.insert(self.item1)
        self.o.insert(self.item2)
        self.o.insert(self.item3)
        dists, data = self.o.get_nearest((26, 30, 40))
        assert data[0].contents == [self.item2]

    def test_get_nearest_point_same(self):
        self.o.insert(self.item1)
        self.o.insert(self.item2)
        self.o.insert(self.item3)
        dists, data = self.o.get_nearest(self.item1.coordinates)
        assert dists[0] == 0
        assert data[0].contents == [self.item1]

    def test_get_nearest_point_list(self):
        self.o.insert(self.item1)
        self.o.insert(self.item2)
        self.o.insert(self.item3)
        self.o.insert(self.item1_copy)
        dists, data = self.o.get_nearest(self.item1.coordinates)
        assert data[0].contents == [self.item1, self.item1_copy]

    def test_get_nearest_two_items(self):
        self.o.insert(self.item1)
        self.o.insert(self.item2)
        self.o.insert(self.item3)
        self.o.insert(self.item1_copy)
        dists, data = self.o.get_nearest((32, 30, 40), k=2)
        contents = [data[i].contents for i in range(len(data))]
        assert [self.item2] in contents
        assert [self.item3] in contents

    def test_get_nearest_equidistant_two_items(self):
        self.o.insert(self.item1)
        self.o.insert(self.item3)
        self.o.insert(self.item1_copy)
        dists, data = self.o.get_nearest(self.item2.coordinates, k=2)
        contents = [data[i].contents for i in range(len(data))]
        assert [self.item1, self.item1_copy] in contents
        assert [self.item3] in contents


class TestRemoval(object):
    def setup(self):
        self.o = Octree((0, 0, 0), 100)
        self.item1 = Frame((20, 30, 40), "An item 1")
        self.item1_copy = Frame((20, 30, 40), "An item 1 copy")
        self.item2 = Frame((30, 30, 40), "An item 2")
        self.item3 = Frame((40, 30, 40), "An item 3")

    def test_remove_nonexistent_item(self):
        with pytest.raises(KeyError):
            self.o.remove((0, 1, 2))

    def test_remove_existing_item(self):
        self.o.insert(self.item1)
        self.o.insert(self.item1_copy)
        self.o.insert(self.item2)
        self.o.insert(self.item3)
        data = self.o.get((40, 30, 40))
        self.o.remove((40, 30, 40))
        assert data.contents == []
        assert data.coordinates is None
        assert len(self.o) == 3

    def test_remove_existing_item_from_list(self):
        self.o.insert(self.item1)
        self.o.insert(self.item1_copy)
        self.o.insert(self.item2)
        self.o.insert(self.item3)
        data = self.o.get((20, 30, 40))
        self.o.remove((20, 30, 40))
        assert data.contents == [self.item1]
        assert len(self.o) == 3

    def test_remove_with_clear(self):
        self.o.insert(self.item1)
        self.o.insert(self.item1_copy)
        self.o.insert(self.item2)
        self.o.insert(self.item3)
        data = self.o.get((20, 30, 40))
        self.o.remove((20, 30, 40), clear=True)
        assert data.contents == []
        assert len(self.o) == 2


class TestPointGathering(object):
    def setup(self):
        self.o = Octree((0, 0, 0), 100)
        self.item1 = Frame((20, 30, 40), "An item 1")
        self.item1_copy = Frame((20, 30, 40), "An item 1 copy")
        self.item2 = Frame((30, 30, 40), "An item 2")
        self.item3 = Frame((40, 30, 40), "An item 3")

    def test_get_points_in_box_empty(self):
        points = self.o.get_points_in_box((0, 0, 0), (25, 35, 50))
        assert list(points) == []

    def test_get_points_in_box_pair(self):
        self.o.insert(self.item1)
        self.o.insert(self.item1_copy)
        points = self.o.get_points_in_box((0, 0, 0), (25, 35, 50))
        contents = [i.contents for i in points]
        assert contents == [[self.item1, self.item1_copy]]

    def test_get_points_in_box_list(self):
        self.o.insert(self.item1)
        self.o.insert(self.item1_copy)
        self.o.insert(self.item2)
        self.o.insert(self.item3)
        points = self.o.get_points_in_box((0, 0, 0), (25, 35, 50))
        contents = [i.contents for i in points]
        assert contents == [[self.item1, self.item1_copy]]

    def test_get_points_in_box_many_items(self):
        self.o.insert(self.item1)
        self.o.insert(self.item1_copy)
        self.o.insert(self.item2)
        self.o.insert(self.item3)
        points = self.o.get_points_in_box((0, 0, 0), (35, 35, 50))
        contents = [i.contents for i in points]
        assert [self.item1, self.item1_copy] in contents
        assert [self.item2] in contents

    def _test_get_nearest_neighbour(self):
        self.o.insert(self.item1)
        self.o.insert(self.item1_copy)
        self.o.insert(self.item2)
        self.o.insert(self.item3)
        nearest = self.o.get_nearest((15, 30, 35))
        assert nearest == self.o.get((20, 30, 40))
