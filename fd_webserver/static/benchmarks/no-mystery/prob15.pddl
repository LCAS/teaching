(define
  (problem strips-mysty-x-15)
  (:domain no-mystery-strips)
  (:objects emmendingen auggen haltingen bad-bellingen denzlingen
      hugstetten tumringen guendlingen kleinkems wollbach bahlingen
      koendringen schallstadt weil schopfheim boetzingen freiburg
      sulki motorrad motorroller kuebelwagen bollerwagen fahrrad
      segway betonmischer daemonenrikscha droschke pferdetransport
      eisbein schlagobers fussball saumagen ochsencremesuppe
      kuechenmaschine halbgefrorenes feinkost-bratling zuckerstange
      faschiertes kiste-bier doener-mit-scharf zamomin fuel-0 fuel-1
      fuel-2 fuel-3 fuel-4 fuel-5 fuel-6 fuel-7 fuel-8 fuel-9 fuel-10
      capacity-0 capacity-1 capacity-2 capacity-3)
  (:init
    (at betonmischer weil)
    (at bollerwagen kleinkems)
    (at daemonenrikscha schopfheim)
    (at doener-mit-scharf boetzingen)
    (at droschke boetzingen)
    (at eisbein emmendingen)
    (at fahrrad wollbach)
    (at faschiertes koendringen)
    (at feinkost-bratling kleinkems)
    (at fussball haltingen)
    (at halbgefrorenes tumringen)
    (at kiste-bier koendringen)
    (at kuebelwagen tumringen)
    (at kuechenmaschine tumringen)
    (at motorrad auggen)
    (at motorroller bad-bellingen)
    (at ochsencremesuppe denzlingen)
    (at pferdetransport freiburg)
    (at saumagen denzlingen)
    (at schlagobers haltingen)
    (at segway bahlingen)
    (at sulki emmendingen)
    (at zamomin freiburg)
    (at zuckerstange wollbach)
    (capacity betonmischer capacity-3)
    (capacity bollerwagen capacity-3)
    (capacity daemonenrikscha capacity-3)
    (capacity droschke capacity-3)
    (capacity fahrrad capacity-2)
    (capacity kuebelwagen capacity-2)
    (capacity motorrad capacity-3)
    (capacity motorroller capacity-1)
    (capacity pferdetransport capacity-1)
    (capacity segway capacity-1)
    (capacity sulki capacity-1)
    (capacity-number capacity-0)
    (capacity-number capacity-1)
    (capacity-number capacity-2)
    (capacity-number capacity-3)
    (capacity-predecessor capacity-0 capacity-1)
    (capacity-predecessor capacity-1 capacity-2)
    (capacity-predecessor capacity-2 capacity-3)
    (connected auggen bad-bellingen)
    (connected auggen denzlingen)
    (connected auggen hugstetten)
    (connected bad-bellingen auggen)
    (connected bad-bellingen denzlingen)
    (connected bad-bellingen guendlingen)
    (connected bad-bellingen haltingen)
    (connected bahlingen koendringen)
    (connected bahlingen weil)
    (connected boetzingen schopfheim)
    (connected boetzingen wollbach)
    (connected denzlingen auggen)
    (connected denzlingen bad-bellingen)
    (connected denzlingen hugstetten)
    (connected emmendingen haltingen)
    (connected emmendingen hugstetten)
    (connected freiburg schallstadt)
    (connected freiburg weil)
    (connected guendlingen bad-bellingen)
    (connected guendlingen tumringen)
    (connected guendlingen weil)
    (connected haltingen bad-bellingen)
    (connected haltingen emmendingen)
    (connected hugstetten auggen)
    (connected hugstetten denzlingen)
    (connected hugstetten emmendingen)
    (connected kleinkems schopfheim)
    (connected kleinkems tumringen)
    (connected koendringen bahlingen)
    (connected koendringen schallstadt)
    (connected schallstadt freiburg)
    (connected schallstadt koendringen)
    (connected schallstadt wollbach)
    (connected schopfheim boetzingen)
    (connected schopfheim kleinkems)
    (connected tumringen guendlingen)
    (connected tumringen kleinkems)
    (connected weil bahlingen)
    (connected weil freiburg)
    (connected weil guendlingen)
    (connected wollbach boetzingen)
    (connected wollbach schallstadt)
    (fuel auggen fuel-6)
    (fuel bad-bellingen fuel-10)
    (fuel bahlingen fuel-2)
    (fuel boetzingen fuel-2)
    (fuel denzlingen fuel-7)
    (fuel emmendingen fuel-1)
    (fuel freiburg fuel-1)
    (fuel guendlingen fuel-5)
    (fuel haltingen fuel-4)
    (fuel hugstetten fuel-4)
    (fuel kleinkems fuel-3)
    (fuel koendringen fuel-4)
    (fuel schallstadt fuel-6)
    (fuel schopfheim fuel-3)
    (fuel tumringen fuel-3)
    (fuel weil fuel-3)
    (fuel wollbach fuel-5)
    (fuel-number fuel-0)
    (fuel-number fuel-1)
    (fuel-number fuel-10)
    (fuel-number fuel-2)
    (fuel-number fuel-3)
    (fuel-number fuel-4)
    (fuel-number fuel-5)
    (fuel-number fuel-6)
    (fuel-number fuel-7)
    (fuel-number fuel-8)
    (fuel-number fuel-9)
    (fuel-predecessor fuel-0 fuel-1)
    (fuel-predecessor fuel-1 fuel-2)
    (fuel-predecessor fuel-2 fuel-3)
    (fuel-predecessor fuel-3 fuel-4)
    (fuel-predecessor fuel-4 fuel-5)
    (fuel-predecessor fuel-5 fuel-6)
    (fuel-predecessor fuel-6 fuel-7)
    (fuel-predecessor fuel-7 fuel-8)
    (fuel-predecessor fuel-8 fuel-9)
    (fuel-predecessor fuel-9 fuel-10)
    (location auggen)
    (location bad-bellingen)
    (location bahlingen)
    (location boetzingen)
    (location denzlingen)
    (location emmendingen)
    (location freiburg)
    (location guendlingen)
    (location haltingen)
    (location hugstetten)
    (location kleinkems)
    (location koendringen)
    (location schallstadt)
    (location schopfheim)
    (location tumringen)
    (location weil)
    (location wollbach)
    (package doener-mit-scharf)
    (package eisbein)
    (package faschiertes)
    (package feinkost-bratling)
    (package fussball)
    (package halbgefrorenes)
    (package kiste-bier)
    (package kuechenmaschine)
    (package ochsencremesuppe)
    (package saumagen)
    (package schlagobers)
    (package zamomin)
    (package zuckerstange)
    (vehicle betonmischer)
    (vehicle bollerwagen)
    (vehicle daemonenrikscha)
    (vehicle droschke)
    (vehicle fahrrad)
    (vehicle kuebelwagen)
    (vehicle motorrad)
    (vehicle motorroller)
    (vehicle pferdetransport)
    (vehicle segway)
    (vehicle sulki))
  (:goal
    (and
      (at feinkost-bratling denzlingen))))