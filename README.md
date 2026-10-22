# Skyrim Vertical Translation (SVT)

[简体中文](README.zh-CN.md)

## An accidental project

This mod took a fairly roundabout path. Most of the time, I make plugins because I need something. SVT was a complete accident.

I've been working on a mod to improve locomotion for close to four months, I think. Somewhere along the way, I ended up working on this instead. I've honestly forgotten how that happened.

Strictly speaking, SVT isn't an AMR replacement. It doesn't implement AMR's functionality. In practice, though, it could very well end up taking its place, because AMR implements a feature the game already has: root motion.

## Skyrim's three root motion paths

It turns out the game has three different ways of handling root motion:

1. **BGS's extracted motion data.** BGS extracts root motion from animations and stores it in `animationdatasinglefile.txt`. The behavior graph decides whether a particular animation uses that data. In this path, the root motion data is stored separately rather than carried by the animation itself.
2. **Paired animations.** The behavior graph can tell the actor to use root motion from the animation itself. Here, the animation carries its own root motion data.
3. **Certain dragon animations.**

I went through `animationdatasinglefile.txt` to find out which vanilla animations had Z-axis movement. The answer was a very small number of dragon animations.

Those animations are unusual. The animation itself has zero net root displacement: there may be a little root movement during playback, but it cancels out by the end. The extracted motion data is attached to the animation as an `hkaDefaultAnimatedReferenceFrame`.

That's Havok's standard approach: run **Extract Motion** on a Havok animation that originally contains root motion, then keep the extracted data with the animation.

That left me wondering why the same animation needed both sets of data. Was the Havok path actually doing anything, or was this just leftover data nobody had removed?

After a lot of behavior experiments, I came to the conclusion that Havok staff probably helped BGS put the dragon behaviors together. That's my inference from the experiments. The game still has the complete implementation: any creature can enable root motion through the same route, and it takes priority over BGS's root motion path.

## AMR and HCT

As for why AMR came along, I honestly don't know. I was away from the modding scene for a long stretch between 2021 and AMR's arrival.

AMR is a hybrid of the BGS and Havok approaches. My guess is that there were two reasons for going that route:

1. alexsylex probably hadn't come across the animations with ReferenceFrames, so there would've been no reason to know this path existed.
2. Extract Motion needs HCT (Havok Content Tools). That can be a hassle for animation authors, and redistributing HCT may raise copyright issues.

A small Blender add-on, or a similar tool that replaces the Extract Motion step, would be a more practical solution.

## Migration and the speed issue

With all those animations already using AMR, I'm honestly not sure how much reason there is to move to the Havok approach. I only care about 1.5.97; later AMR development is outside the scope of this project.

For 1.5.97, replacing AMR would basically come down to an automated tool that reads its annotations in bulk and converts them into equivalent ReferenceFrame data. Another option would be to convert the vanilla `animationdatasinglefile.txt` data into equivalent ReferenceFrames.

The behavior patches would then need to use `bIsSynced` in place of `bAllowRotation` or `bAnimationDriven`. Only a few mods, such as BFCO and MCO, would need that change.

I haven't built that batch conversion tool, though. The game doesn't process equivalent data from `animationdatasinglefile.txt` and ReferenceFrames in quite the same way. Even with identical animations and displacement, the speed calculated for the actor can differ. An incorrect speed at the end of an animation can trigger a walking or running animation.

In my observations, the actor's calculated speed jumps around a lot during playback with the BGS approach. With the Havok approach, it's very stable and barely fluctuates.

A straight conversion of the vanilla data left a small, nonzero speed at the end of the animation. That was enough to produce a one-frame flash of a locomotion animation.

I haven't seen that problem when converting AMR data to the Havok format. In the end, I didn't make the converter because I was lazy. Replacing AMR was never my goal, and this plugin doesn't implement root motion itself anyway.

## What SVT actually does

**SVT stops the vanilla game from blocking human Z-axis root motion.** That part turned out to be surprisingly complicated. The game has a lot of restrictions intended to keep characters on the ground.

My first plan was to find out how dragons handled vertical movement and have humans do the same thing. But dragons are in the `Flying` state, while those restrictions only apply in `OnGround`.

I suspect alexsylex also approached Z-axis movement by looking for a way to change states. There is also an AMR build from alexsylex with Z-axis support.

## The `Flying` experiment

I initially tried using `Flying` to get around those restrictions, and it worked extremely well with very little code. The game even handled the state transition automatically once the animation finished.

The problem was that collision didn't work properly in that state.

I spent a lot of time reverse-engineering the dragon code, but I couldn't find the collision handling I was looking for in `Flying`. I was sure there had to be collision handling somewhere; what I actually found was mostly code for dragons actively avoiding obstacles.

Here, when I say "true flight," I mean the game's `Flying` state. Mods advertising true flight with collision aren't actually using that state. The character is in a different one.

I also looked at Loki's Paraglider, because I remembered it had a way to take off from flat ground. My conclusion was that these mods were mostly using clever workarounds.

Eventually, I gave up on the `Flying` approach. It felt great in practice, but I didn't want to spend a huge amount of time figuring out how to add proper collision.

## Where I left it

In the end, I handed the pile of junk I'd put together to an AI and had it make an alternative implementation.

It works pretty well. At least, I haven't found anything wrong with it during my own review.

I might redo this mod someday, but it isn't where I'm focusing my development time.

(I don't even use mods with Z-axis root motion myself.)
