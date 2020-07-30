<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.6.0">
<drawing>
<settings>
<setting alwaysvectorfont="yes"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="2" name="Route2" color="16" fill="1" visible="no" active="no"/>
<layer number="3" name="Route3" color="17" fill="1" visible="no" active="no"/>
<layer number="4" name="Route4" color="18" fill="1" visible="no" active="no"/>
<layer number="5" name="Route5" color="19" fill="1" visible="no" active="no"/>
<layer number="6" name="Route6" color="25" fill="1" visible="no" active="no"/>
<layer number="7" name="Route7" color="26" fill="1" visible="no" active="no"/>
<layer number="8" name="Route8" color="27" fill="1" visible="no" active="no"/>
<layer number="9" name="Route9" color="28" fill="1" visible="no" active="no"/>
<layer number="10" name="Route10" color="29" fill="1" visible="no" active="no"/>
<layer number="11" name="Route11" color="30" fill="1" visible="no" active="no"/>
<layer number="12" name="Route12" color="20" fill="1" visible="no" active="no"/>
<layer number="13" name="Route13" color="21" fill="1" visible="no" active="no"/>
<layer number="14" name="Route14" color="22" fill="1" visible="no" active="no"/>
<layer number="15" name="Route15" color="23" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="24" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="99" name="SpiceOrder" color="5" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="ma732_adapter">
<packages>
<package name="QFN16" urn="urn:adsk.eagle:footprint:21050/1">
<description>&lt;b&gt;QFN 16&lt;/b&gt; 3x3 mm&lt;p&gt;
Source: www.hittite.com .. hmc492lp3.pdf</description>
<wire x1="-1.5" y1="1.5" x2="1.5" y2="1.5" width="0.1016" layer="51"/>
<wire x1="1.5" y1="1.5" x2="1.5" y2="-1.5" width="0.1016" layer="51"/>
<wire x1="1.5" y1="-1.5" x2="-1.5" y2="-1.5" width="0.1016" layer="51"/>
<wire x1="-1.5" y1="-1.5" x2="-1.5" y2="1.5" width="0.1016" layer="51"/>
<wire x1="1.5" y1="1.5" x2="1.5" y2="1" width="0.1016" layer="21"/>
<wire x1="1" y1="1.5" x2="1.5" y2="1.5" width="0.1016" layer="21"/>
<wire x1="-1.5" y1="1.5" x2="-1" y2="1.5" width="0.1016" layer="21"/>
<wire x1="-1.5" y1="1" x2="-1.5" y2="1.5" width="0.1016" layer="21"/>
<wire x1="-1.5" y1="-1.5" x2="-1.5" y2="-1" width="0.1016" layer="21"/>
<wire x1="-1" y1="-1.5" x2="-1.5" y2="-1.5" width="0.1016" layer="21"/>
<wire x1="1.5" y1="-1.5" x2="1" y2="-1.5" width="0.1016" layer="21"/>
<wire x1="1.5" y1="-1" x2="1.5" y2="-1.5" width="0.1016" layer="21"/>
<circle x="-1.25" y="1.25" radius="0.152" width="0" layer="21"/>
<smd name="EXP" x="0" y="0" dx="1.95" dy="1.95" layer="1" stop="no" cream="no"/>
<smd name="1" x="-1.4" y="0.75" dx="0.5" dy="0.3" layer="1" stop="no" cream="no"/>
<smd name="2" x="-1.4" y="0.25" dx="0.5" dy="0.3" layer="1" stop="no" cream="no"/>
<smd name="3" x="-1.4" y="-0.25" dx="0.5" dy="0.3" layer="1" stop="no" cream="no"/>
<smd name="4" x="-1.4" y="-0.75" dx="0.5" dy="0.3" layer="1" stop="no" cream="no"/>
<smd name="5" x="-0.75" y="-1.4" dx="0.5" dy="0.3" layer="1" rot="R90" stop="no" cream="no"/>
<smd name="6" x="-0.25" y="-1.4" dx="0.5" dy="0.3" layer="1" rot="R90" stop="no" cream="no"/>
<smd name="7" x="0.25" y="-1.4" dx="0.5" dy="0.3" layer="1" rot="R90" stop="no" cream="no"/>
<smd name="8" x="0.75" y="-1.4" dx="0.5" dy="0.3" layer="1" rot="R90" stop="no" cream="no"/>
<smd name="9" x="1.4" y="-0.75" dx="0.5" dy="0.3" layer="1" rot="R180" stop="no" cream="no"/>
<smd name="10" x="1.4" y="-0.25" dx="0.5" dy="0.3" layer="1" rot="R180" stop="no" cream="no"/>
<smd name="11" x="1.4" y="0.25" dx="0.5" dy="0.3" layer="1" rot="R180" stop="no" cream="no"/>
<smd name="12" x="1.4" y="0.75" dx="0.5" dy="0.3" layer="1" rot="R180" stop="no" cream="no"/>
<smd name="13" x="0.75" y="1.4" dx="0.5" dy="0.3" layer="1" rot="R270" stop="no" cream="no"/>
<smd name="14" x="0.25" y="1.4" dx="0.5" dy="0.3" layer="1" rot="R270" stop="no" cream="no"/>
<smd name="15" x="-0.25" y="1.4" dx="0.5" dy="0.3" layer="1" rot="R270" stop="no" cream="no"/>
<smd name="16" x="-0.75" y="1.4" dx="0.5" dy="0.3" layer="1" rot="R270" stop="no" cream="no"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1" y1="-1" x2="1" y2="1" layer="29"/>
<rectangle x1="-0.925" y1="-0.925" x2="0.925" y2="0.925" layer="31"/>
<rectangle x1="-1.675" y1="0.575" x2="-1.125" y2="0.925" layer="29"/>
<rectangle x1="-1.6" y1="0.625" x2="-1.175" y2="0.875" layer="31"/>
<rectangle x1="-1.675" y1="0.075" x2="-1.125" y2="0.425" layer="29"/>
<rectangle x1="-1.6" y1="0.125" x2="-1.175" y2="0.375" layer="31"/>
<rectangle x1="-1.675" y1="-0.425" x2="-1.125" y2="-0.075" layer="29"/>
<rectangle x1="-1.6" y1="-0.375" x2="-1.175" y2="-0.125" layer="31"/>
<rectangle x1="-1.675" y1="-0.925" x2="-1.125" y2="-0.575" layer="29"/>
<rectangle x1="-1.6" y1="-0.875" x2="-1.175" y2="-0.625" layer="31"/>
<rectangle x1="-1.025" y1="-1.575" x2="-0.475" y2="-1.225" layer="29" rot="R90"/>
<rectangle x1="-0.9625" y1="-1.5125" x2="-0.5375" y2="-1.2625" layer="31" rot="R90"/>
<rectangle x1="-0.525" y1="-1.575" x2="0.025" y2="-1.225" layer="29" rot="R90"/>
<rectangle x1="-0.4625" y1="-1.5125" x2="-0.0375" y2="-1.2625" layer="31" rot="R90"/>
<rectangle x1="-0.025" y1="-1.575" x2="0.525" y2="-1.225" layer="29" rot="R90"/>
<rectangle x1="0.0375" y1="-1.5125" x2="0.4625" y2="-1.2625" layer="31" rot="R90"/>
<rectangle x1="0.475" y1="-1.575" x2="1.025" y2="-1.225" layer="29" rot="R90"/>
<rectangle x1="0.5375" y1="-1.5125" x2="0.9625" y2="-1.2625" layer="31" rot="R90"/>
<rectangle x1="1.125" y1="-0.925" x2="1.675" y2="-0.575" layer="29" rot="R180"/>
<rectangle x1="1.175" y1="-0.875" x2="1.6" y2="-0.625" layer="31" rot="R180"/>
<rectangle x1="1.125" y1="-0.425" x2="1.675" y2="-0.075" layer="29" rot="R180"/>
<rectangle x1="1.175" y1="-0.375" x2="1.6" y2="-0.125" layer="31" rot="R180"/>
<rectangle x1="1.125" y1="0.075" x2="1.675" y2="0.425" layer="29" rot="R180"/>
<rectangle x1="1.175" y1="0.125" x2="1.6" y2="0.375" layer="31" rot="R180"/>
<rectangle x1="1.125" y1="0.575" x2="1.675" y2="0.925" layer="29" rot="R180"/>
<rectangle x1="1.175" y1="0.625" x2="1.6" y2="0.875" layer="31" rot="R180"/>
<rectangle x1="0.475" y1="1.225" x2="1.025" y2="1.575" layer="29" rot="R270"/>
<rectangle x1="0.5375" y1="1.2625" x2="0.9625" y2="1.5125" layer="31" rot="R270"/>
<rectangle x1="-0.025" y1="1.225" x2="0.525" y2="1.575" layer="29" rot="R270"/>
<rectangle x1="0.0375" y1="1.2625" x2="0.4625" y2="1.5125" layer="31" rot="R270"/>
<rectangle x1="-0.525" y1="1.225" x2="0.025" y2="1.575" layer="29" rot="R270"/>
<rectangle x1="-0.4625" y1="1.2625" x2="-0.0375" y2="1.5125" layer="31" rot="R270"/>
<rectangle x1="-1.025" y1="1.225" x2="-0.475" y2="1.575" layer="29" rot="R270"/>
<rectangle x1="-0.9625" y1="1.2625" x2="-0.5375" y2="1.5125" layer="31" rot="R270"/>
</package>
<package name="TSSOP_PAD">
<pad name="P$1" x="0" y="0" drill="0.254"/>
</package>
</packages>
<symbols>
<symbol name="QFN16-BO">
<wire x1="-12.7" y1="12.7" x2="-12.7" y2="-12.7" width="0.254" layer="94"/>
<wire x1="-12.7" y1="-12.7" x2="12.7" y2="-12.7" width="0.254" layer="94"/>
<wire x1="12.7" y1="-12.7" x2="12.7" y2="12.7" width="0.254" layer="94"/>
<wire x1="12.7" y1="12.7" x2="-12.7" y2="12.7" width="0.254" layer="94"/>
<pin name="1" x="-17.78" y="7.62" length="middle"/>
<pin name="2" x="-17.78" y="2.54" length="middle"/>
<pin name="3" x="-17.78" y="-2.54" length="middle"/>
<pin name="4" x="-17.78" y="-7.62" length="middle"/>
<pin name="5" x="-7.62" y="-17.78" length="middle" rot="R90"/>
<pin name="6" x="-2.54" y="-17.78" length="middle" rot="R90"/>
<pin name="7" x="2.54" y="-17.78" length="middle" rot="R90"/>
<pin name="8" x="7.62" y="-17.78" length="middle" rot="R90"/>
<pin name="9" x="17.78" y="-7.62" length="middle" rot="R180"/>
<pin name="10" x="17.78" y="-2.54" length="middle" rot="R180"/>
<pin name="11" x="17.78" y="2.54" length="middle" rot="R180"/>
<pin name="12" x="17.78" y="7.62" length="middle" rot="R180"/>
<pin name="13" x="7.62" y="17.78" length="middle" rot="R270"/>
<pin name="14" x="2.54" y="17.78" length="middle" rot="R270"/>
<pin name="15" x="-2.54" y="17.78" length="middle" rot="R270"/>
<pin name="16" x="-7.62" y="17.78" length="middle" rot="R270"/>
<pin name="PAD" x="17.78" y="-12.7" length="middle" rot="R180"/>
</symbol>
<symbol name="TSSOP_PAD">
<pin name="P$1" x="-5.08" y="0" visible="off" length="middle"/>
<text x="-7.62" y="2.54" size="1.27" layer="94">&gt;NAME</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="QFN16-BO">
<gates>
<gate name="G$1" symbol="QFN16-BO" x="0" y="0"/>
</gates>
<devices>
<device name="" package="QFN16">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="10" pad="10"/>
<connect gate="G$1" pin="11" pad="11"/>
<connect gate="G$1" pin="12" pad="12"/>
<connect gate="G$1" pin="13" pad="13"/>
<connect gate="G$1" pin="14" pad="14"/>
<connect gate="G$1" pin="15" pad="15"/>
<connect gate="G$1" pin="16" pad="16"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
<connect gate="G$1" pin="7" pad="7"/>
<connect gate="G$1" pin="8" pad="8"/>
<connect gate="G$1" pin="9" pad="9"/>
<connect gate="G$1" pin="PAD" pad="EXP"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="TSSOP_PAD">
<gates>
<gate name="G$1" symbol="TSSOP_PAD" x="0" y="0"/>
</gates>
<devices>
<device name="" package="TSSOP_PAD">
<connects>
<connect gate="G$1" pin="P$1" pad="P$1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="U$1" library="ma732_adapter" deviceset="QFN16-BO" device=""/>
<part name="CSN" library="ma732_adapter" deviceset="TSSOP_PAD" device=""/>
<part name="CLK" library="ma732_adapter" deviceset="TSSOP_PAD" device=""/>
<part name="MISO" library="ma732_adapter" deviceset="TSSOP_PAD" device=""/>
<part name="MOSI" library="ma732_adapter" deviceset="TSSOP_PAD" device=""/>
<part name="TEST" library="ma732_adapter" deviceset="TSSOP_PAD" device=""/>
<part name="B" library="ma732_adapter" deviceset="TSSOP_PAD" device=""/>
<part name="A" library="ma732_adapter" deviceset="TSSOP_PAD" device=""/>
<part name="PWM" library="ma732_adapter" deviceset="TSSOP_PAD" device=""/>
<part name="GND" library="ma732_adapter" deviceset="TSSOP_PAD" device=""/>
<part name="VDD3V" library="ma732_adapter" deviceset="TSSOP_PAD" device=""/>
<part name="VDD" library="ma732_adapter" deviceset="TSSOP_PAD" device=""/>
<part name="U" library="ma732_adapter" deviceset="TSSOP_PAD" device=""/>
<part name="V" library="ma732_adapter" deviceset="TSSOP_PAD" device=""/>
<part name="W" library="ma732_adapter" deviceset="TSSOP_PAD" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="U$1" gate="G$1" x="68.58" y="48.26" smashed="yes"/>
<instance part="CSN" gate="G$1" x="30.48" y="66.04" smashed="yes">
<attribute name="NAME" x="22.86" y="68.58" size="1.27" layer="94"/>
</instance>
<instance part="CLK" gate="G$1" x="30.48" y="58.42" smashed="yes">
<attribute name="NAME" x="22.86" y="60.96" size="1.27" layer="94"/>
</instance>
<instance part="MISO" gate="G$1" x="30.48" y="50.8" smashed="yes">
<attribute name="NAME" x="22.86" y="53.34" size="1.27" layer="94"/>
</instance>
<instance part="MOSI" gate="G$1" x="30.48" y="43.18" smashed="yes">
<attribute name="NAME" x="22.86" y="45.72" size="1.27" layer="94"/>
</instance>
<instance part="TEST" gate="G$1" x="30.48" y="35.56" smashed="yes">
<attribute name="NAME" x="22.86" y="38.1" size="1.27" layer="94"/>
</instance>
<instance part="B" gate="G$1" x="30.48" y="27.94" smashed="yes">
<attribute name="NAME" x="22.86" y="30.48" size="1.27" layer="94"/>
</instance>
<instance part="A" gate="G$1" x="30.48" y="20.32" smashed="yes">
<attribute name="NAME" x="22.86" y="22.86" size="1.27" layer="94"/>
</instance>
<instance part="PWM" gate="G$1" x="106.68" y="66.04" smashed="yes" rot="R180">
<attribute name="NAME" x="114.3" y="63.5" size="1.27" layer="94" rot="R180"/>
</instance>
<instance part="GND" gate="G$1" x="106.68" y="58.42" smashed="yes" rot="R180">
<attribute name="NAME" x="114.3" y="55.88" size="1.27" layer="94" rot="R180"/>
</instance>
<instance part="VDD3V" gate="G$1" x="106.68" y="50.8" smashed="yes" rot="R180">
<attribute name="NAME" x="114.3" y="48.26" size="1.27" layer="94" rot="R180"/>
</instance>
<instance part="VDD" gate="G$1" x="106.68" y="43.18" smashed="yes" rot="R180">
<attribute name="NAME" x="114.3" y="40.64" size="1.27" layer="94" rot="R180"/>
</instance>
<instance part="U" gate="G$1" x="106.68" y="35.56" smashed="yes" rot="R180">
<attribute name="NAME" x="114.3" y="33.02" size="1.27" layer="94" rot="R180"/>
</instance>
<instance part="V" gate="G$1" x="106.68" y="27.94" smashed="yes" rot="R180">
<attribute name="NAME" x="114.3" y="25.4" size="1.27" layer="94" rot="R180"/>
</instance>
<instance part="W" gate="G$1" x="106.68" y="20.32" smashed="yes" rot="R180">
<attribute name="NAME" x="114.3" y="17.78" size="1.27" layer="94" rot="R180"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="N$15" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="5"/>
<wire x1="60.96" y1="30.48" x2="58.42" y2="30.48" width="0.1524" layer="91"/>
<wire x1="58.42" y1="30.48" x2="58.42" y2="22.86" width="0.1524" layer="91"/>
<wire x1="58.42" y1="22.86" x2="40.64" y2="22.86" width="0.1524" layer="91"/>
<wire x1="40.64" y1="22.86" x2="40.64" y2="66.04" width="0.1524" layer="91"/>
<pinref part="CSN" gate="G$1" pin="P$1"/>
<wire x1="40.64" y1="66.04" x2="25.4" y2="66.04" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$16" class="0">
<segment>
<pinref part="CLK" gate="G$1" pin="P$1"/>
<wire x1="25.4" y1="58.42" x2="45.72" y2="58.42" width="0.1524" layer="91"/>
<wire x1="45.72" y1="58.42" x2="45.72" y2="73.66" width="0.1524" layer="91"/>
<wire x1="45.72" y1="73.66" x2="91.44" y2="73.66" width="0.1524" layer="91"/>
<wire x1="91.44" y1="73.66" x2="91.44" y2="55.88" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="12"/>
<wire x1="91.44" y1="55.88" x2="86.36" y2="55.88" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$17" class="0">
<segment>
<pinref part="MISO" gate="G$1" pin="P$1"/>
<wire x1="25.4" y1="50.8" x2="45.72" y2="50.8" width="0.1524" layer="91"/>
<wire x1="45.72" y1="50.8" x2="45.72" y2="17.78" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="7"/>
<wire x1="45.72" y1="17.78" x2="71.12" y2="17.78" width="0.1524" layer="91"/>
<wire x1="71.12" y1="17.78" x2="71.12" y2="30.48" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$18" class="0">
<segment>
<pinref part="MOSI" gate="G$1" pin="P$1"/>
<wire x1="25.4" y1="43.18" x2="35.56" y2="43.18" width="0.1524" layer="91"/>
<wire x1="35.56" y1="43.18" x2="35.56" y2="40.64" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="4"/>
<wire x1="35.56" y1="40.64" x2="50.8" y2="40.64" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$19" class="0">
<segment>
<pinref part="TEST" gate="G$1" pin="P$1"/>
<wire x1="25.4" y1="35.56" x2="15.24" y2="35.56" width="0.1524" layer="91"/>
<wire x1="15.24" y1="35.56" x2="15.24" y2="10.16" width="0.1524" layer="91"/>
<wire x1="15.24" y1="10.16" x2="91.44" y2="10.16" width="0.1524" layer="91"/>
<wire x1="91.44" y1="10.16" x2="91.44" y2="45.72" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="10"/>
<wire x1="91.44" y1="45.72" x2="86.36" y2="45.72" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$20" class="0">
<segment>
<pinref part="B" gate="G$1" pin="P$1"/>
<wire x1="25.4" y1="27.94" x2="20.32" y2="27.94" width="0.1524" layer="91"/>
<wire x1="20.32" y1="27.94" x2="20.32" y2="12.7" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="6"/>
<wire x1="20.32" y1="12.7" x2="66.04" y2="12.7" width="0.1524" layer="91"/>
<wire x1="66.04" y1="12.7" x2="66.04" y2="30.48" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$21" class="0">
<segment>
<pinref part="A" gate="G$1" pin="P$1"/>
<wire x1="25.4" y1="20.32" x2="22.86" y2="20.32" width="0.1524" layer="91"/>
<wire x1="22.86" y1="20.32" x2="22.86" y2="17.78" width="0.1524" layer="91"/>
<wire x1="22.86" y1="17.78" x2="38.1" y2="17.78" width="0.1524" layer="91"/>
<wire x1="38.1" y1="17.78" x2="38.1" y2="53.34" width="0.1524" layer="91"/>
<wire x1="38.1" y1="53.34" x2="48.26" y2="53.34" width="0.1524" layer="91"/>
<wire x1="48.26" y1="53.34" x2="48.26" y2="50.8" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="2"/>
<wire x1="48.26" y1="50.8" x2="50.8" y2="50.8" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$22" class="0">
<segment>
<pinref part="PWM" gate="G$1" pin="P$1"/>
<wire x1="111.76" y1="66.04" x2="119.38" y2="66.04" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="9"/>
<wire x1="119.38" y1="66.04" x2="119.38" y2="40.64" width="0.1524" layer="91"/>
<wire x1="119.38" y1="40.64" x2="86.36" y2="40.64" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$23" class="0">
<segment>
<pinref part="GND" gate="G$1" pin="P$1"/>
<wire x1="111.76" y1="58.42" x2="127" y2="58.42" width="0.1524" layer="91"/>
<wire x1="127" y1="58.42" x2="127" y2="25.4" width="0.1524" layer="91"/>
<wire x1="127" y1="25.4" x2="76.2" y2="25.4" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="8"/>
<wire x1="76.2" y1="25.4" x2="76.2" y2="30.48" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="PAD"/>
<wire x1="86.36" y1="35.56" x2="99.06" y2="35.56" width="0.1524" layer="91"/>
<wire x1="99.06" y1="35.56" x2="99.06" y2="58.42" width="0.1524" layer="91"/>
<wire x1="99.06" y1="58.42" x2="111.76" y2="58.42" width="0.1524" layer="91"/>
<junction x="111.76" y="58.42"/>
</segment>
</net>
<net name="N$24" class="0">
<segment>
<pinref part="VDD3V" gate="G$1" pin="P$1"/>
<wire x1="111.76" y1="50.8" x2="124.46" y2="50.8" width="0.1524" layer="91"/>
<wire x1="124.46" y1="50.8" x2="124.46" y2="78.74" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="13"/>
<wire x1="124.46" y1="78.74" x2="76.2" y2="78.74" width="0.1524" layer="91"/>
<wire x1="76.2" y1="78.74" x2="76.2" y2="66.04" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
</compatibility>
</eagle>
