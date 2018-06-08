# Curiously recurring template pattern
<!--
// Copyright (c) 2016 CNRS
// Author: Florent Lamiraux, Justin Carpentier, Florian Valenza
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.
-->

Similarly to Eigen, Pinocchio library makes intensive use of the so called CRTP design
pattern. This pattern is used for performance reasons in the
implementation of static polymorphism, avoiding dynamic casts and calls to virtual methods.
All in all, CRTP plays a central role in the performance of Pinocchio.

We refer to  https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern for further explanations.
