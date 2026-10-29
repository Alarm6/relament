import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';

const html=fs.readFileSync(new URL('../dist/index.html',import.meta.url),'utf8');
const solver=html.split('// CAP_SOLVER_BEGIN')[1].split('// CAP_SOLVER_END')[0];
const engine=html.split('<script id="rigid-body-engine">')[1].split('</script>')[0];
const context=vm.createContext({window:{},console,performance});vm.runInContext(engine,context);const create=vm.runInContext(solver+';createCapSimulation',context);
function setup(count=76){
  const bodies=Array.from({length:count},(_,i)=>{const scale=.75+((i*37)%65)/100;return {p:{x:0,y:0,z:0},v:{x:0,y:0,z:0},mass:scale**3,r:.408*scale,baseRadius:.408*scale}});
  const sim=create(bodies);sim.reset(3.15);
  const cfg={bound:3.15,gather:0,feed:false,progress:0,floor:x=>-2.4+(Math.abs(x)/3.15)**3*1.35,mouse:{x:0,y:0,z:0},mouseActive:false,mouseRadius:1.8,mouseSpeed:0};
  return {bodies,sim,cfg};
}
function run(world,seconds,fps=60){for(let i=0;i<seconds*fps;i++)world.sim.advance(1/fps,world.cfg)}
const metrics={};
// Two face-on caps may overlap completely on screen while separated in depth.
const depth=setup(2);depth.cfg.gather=1;depth.cfg.externalForces=false;depth.cfg.floor=()=>-100;
depth.bodies.forEach((b,i)=>{b.p.set(0,0,i? .14:-.14);b.rigid.quaternion.setFromAxisAngle(new context.window.RelamentRigidBody.Vec3(1,0,0),Math.PI/2);b.rigid.aabbNeedsUpdate=true});
assert.equal(depth.sim.contactInfo(...depth.bodies,true).penetration,0);
run(depth,1);metrics.depthOverlap=depth.sim.diagnostics();assert.equal(metrics.depthOverlap.penetration,0);assert.equal(metrics.depthOverlap.projectedOverlaps,1);
// Flat caps can also be stacked with far less than two enclosing-sphere radii.
const stack=setup(2);stack.bodies.forEach((b,i)=>{b.p.set(0,i?.27:0,0);b.rigid.quaternion.set(0,0,0,1)});assert.equal(stack.sim.contactInfo(...stack.bodies,true).penetration,0);
// A genuine surface intersection must still be detected and resolved.
depth.bodies[1].p.z=depth.bodies[0].p.z+.10;depth.bodies.forEach(b=>b.rigid.wakeUp());assert.ok(depth.sim.contactInfo(...depth.bodies,true).penetration>.08);run(depth,2);metrics.correctedSurfaces=depth.sim.diagnostics();assert.ok(metrics.correctedSurfaces.penetration<.006);


// A fully populated pile must settle without intersecting or moving at rest.
const pile=setup();run(pile,16);metrics.pile=pile.sim.diagnostics();
console.log('pile',JSON.stringify(metrics.pile));assert.ok(metrics.pile.penetration<.006,JSON.stringify(metrics.pile));
assert.ok(metrics.pile.sleeping>=60,JSON.stringify(metrics.pile));
const settled=pile.bodies.map(b=>({position:{...b.p},asleep:b.sleeping,woke:false}));pile.bodies.forEach((b,i)=>b.rigid.addEventListener('wakeup',()=>{settled[i].woke=true}));run(pile,2);
// A cap still sliding is allowed to move. A sleeping cap without a new impact is not.
const stable=pile.bodies.filter((b,i)=>settled[i].asleep&&!settled[i].woke);assert.ok(stable.length>=50,`continuously sleeping caps ${stable.length}`);
const restMovement=Math.max(...pile.bodies.map((b,i)=>settled[i].asleep&&!settled[i].woke?Math.hypot(b.p.x-settled[i].position.x,b.p.y-settled[i].position.y,b.p.z-settled[i].position.z):0));
metrics.restMovement=restMovement;metrics.stableSleepingCaps=stable.length;assert.ok(restMovement<1e-8,`sleeping-body movement ${restMovement}`);

// The same physical duration yields the same result at low and high refresh rates.
const low=setup(16),high=setup(16);run(low,8,30);run(high,8,144);
const frameDifference=Math.max(...low.bodies.map((b,i)=>Math.hypot(b.p.x-high.bodies[i].p.x,b.p.y-high.bodies[i].p.y,b.p.z-high.bodies[i].p.z)));
metrics.frameDifference=frameDifference;assert.ok(frameDifference<1e-7,`frame difference ${frameDifference}`);

// Gathering and a strong cursor impulse must retain collision separation.
pile.cfg.gather=1;run(pile,12);metrics.cluster=pile.sim.diagnostics();assert.ok(metrics.cluster.penetration<.006,JSON.stringify(metrics.cluster));
pile.cfg.mouseActive=true;pile.cfg.mouseSpeed=3;run(pile,.5);pile.cfg.mouseActive=false;run(pile,8);metrics.afterFlick=pile.sim.diagnostics();console.log('afterFlick',JSON.stringify(metrics.afterFlick));assert.ok(metrics.afterFlick.penetration<.006,JSON.stringify(metrics.afterFlick));

// Contacts remain active in the feed hopper, and all eligible caps can be consumed.
pile.cfg.feed=true;pile.cfg.progress=.2;for(const b of pile.bodies)b.r=b.baseRadius*.53;run(pile,5);metrics.hopper=pile.sim.diagnostics();assert.ok(metrics.hopper.penetration<.006,JSON.stringify(metrics.hopper));
pile.cfg.progress=1.2;run(pile,12);metrics.finished=pile.sim.diagnostics();assert.equal(metrics.finished.visible,0,JSON.stringify(metrics.finished));
console.log(JSON.stringify(metrics,null,2));
